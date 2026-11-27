"""Build Bambu Studio plate files (.3mf) and print-ready STLs for the Bambu Lab P2S.

    python build_plates.py          # run from cad/print/bambu-p2s after regenerating cad/stl

Each part is turned to its best printing face (largest flat face down, fewest
overhangs), laid out on the P2S's 256 x 256 mm plate, and saved with its
per-part settings (walls, infill, brim, supports) for Bambu Studio.
"""
import sys, os, zipfile, numpy as np, trimesh

# rotations that put each face of the model on the bed
R = {'+z': np.eye(4),
   '-z':trimesh.transformations.rotation_matrix(np.pi,[1,0,0]),
   '+x':trimesh.transformations.rotation_matrix(np.pi/2,[0,1,0]),
   '-x':trimesh.transformations.rotation_matrix(-np.pi/2,[0,1,0]),
   '+y':trimesh.transformations.rotation_matrix(-np.pi/2,[1,0,0]),
   '-y':trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0])}
STL = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'stl') + os.sep

def oriented(fname, key):
    """Load a part, turn it onto the chosen face and move it to the origin."""
    m=trimesh.load(STL+fname); m.apply_transform(R[key])
    m.apply_translation(-m.bounds[0]); return m
BED = 256.0
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))

# part: (stl file, face to put on the bed, Bambu Studio per-object settings)
ASA60 = dict(wall_loops='4', sparse_infill_density='60%', sparse_infill_pattern='gyroid')
ASA40 = dict(wall_loops='4', sparse_infill_density='40%', sparse_infill_pattern='gyroid')
BRIM = dict(brim_type='outer_only', brim_width='5')
SUP = dict(enable_support='1', support_type='tree(auto)')
PARTS = {
 'P1': ('P1_pan_base.stl', '-z', {**ASA60, **BRIM}),
 'P2': ('P2_yoke.stl', '+z', {**ASA60, **BRIM}),
 'P3a': ('P3a_cradle_horn_half.stl', '+y', ASA60),
 'P3b': ('P3b_cradle_axle_half.stl', '-y', ASA60),
 'P4': ('P4_pipe_clamp_half.stl', '+z', ASA40),
 'P5': ('P5_mounting_plate.stl', '+y', {**ASA40, **BRIM}),
 'P6': ('P6_post_clip.stl', '+z', ASA40),
 'P7': ('P7_sensor_bracket.stl', '+z', ASA40),
 'P8': ('P8_thermal_visor.stl', '+y', {**ASA40, **SUP}),
 'P9': ('P9_thermal_carrier.stl', '-y', ASA40),
}
# plate layouts: (part, x, y, rotate 90 about z?)  x,y = min corner on the bed, mm
PLATES = {
 'plate1_ASA_head_P1-P3_P8': [('P1', 6, 6, False), ('P2', 6, 128, False), ('P8', 152, 6, True),
                               ('P3a', 120, 140, False), ('P3b', 120, 180, False)],
 'plate2_ASA_post_P4-P5': [('P5', 6, 6, False), ('P5', 104, 6, False)] +
                          [('P4', x, y, True) for x in (199, 229) for y in (6, 77, 148)] +
                          [('P4', 6, 220, False), ('P4', 80, 220, False)],
 'plate3_PETG_sensor_P6_P7_P9': [('P7', 6, 6, False), ('P9', 130, 6, False)] +
                                [('P6', 6 + 52 * i, 70, False) for i in range(3)] +
                                [('P6', 6 + 52 * i, 125, False) for i in range(3)],
}

def placed(part, x, y, rot):
    f, k, _ = PARTS[part]
    m = oriented(f, k)
    if rot:
        m.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 0, 1]))
        m.apply_translation(-m.bounds[0])
    m.apply_translation([x, y, 0])
    return m

def check(name, items):
    boxes = []
    for part, x, y, rot in items:
        b = placed(part, x, y, rot).bounds
        assert b[0, 0] >= 2 and b[0, 1] >= 2 and b[1, 0] <= BED - 2 and b[1, 1] <= BED - 2 and b[1, 2] <= BED, (name, part, b)
        for p2, b2 in boxes:
            gap = max(b2[0, 0] - b[1, 0], b[0, 0] - b2[1, 0], b2[0, 1] - b[1, 1], b[0, 1] - b2[1, 1])
            assert gap >= 5, (name, part, p2, gap)
        boxes.append((part, b))
    return boxes

def mesh_xml(m):
    v = '\n'.join(f'     <vertex x="{a:.4f}" y="{b:.4f}" z="{c:.4f}"/>' for a, b, c in m.vertices)
    t = '\n'.join(f'     <triangle v1="{i}" v2="{j}" v3="{k}"/>' for i, j, k in m.faces)
    return f'   <mesh>\n    <vertices>\n{v}\n    </vertices>\n    <triangles>\n{t}\n    </triangles>\n   </mesh>'

def write_3mf(path, items):
    objs, build, cfg = [], [], []
    count = {}
    for oid, (part, x, y, rot) in enumerate(items, 1):
        count[part] = count.get(part, 0) + 1
        name = PARTS[part][0][:-4] + (f'_{count[part]}' if sum(1 for it in items if it[0] == part) > 1 else '')
        m = placed(part, x, y, rot)
        objs.append(f'  <object id="{oid}" type="model" name="{name}">\n{mesh_xml(m)}\n  </object>')
        build.append(f'  <item objectid="{oid}" printable="1"/>')
        meta = [('name', name), ('extruder', '1')] + list(PARTS[part][2].items())
        cfg.append(f'  <object id="{oid}">\n' + '\n'.join(f'    <metadata key="{k}" value="{v}"/>' for k, v in meta) + '\n  </object>')
    model = ('<?xml version="1.0" encoding="UTF-8"?>\n'
             '<model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">\n'
             ' <metadata name="Title">Lawn Watering Turret - ' + os.path.basename(path)[:-4] + '</metadata>\n'
             ' <metadata name="Designer">Kaushik Yanamandra</metadata>\n'
             ' <resources>\n' + '\n'.join(objs) + '\n </resources>\n <build>\n' + '\n'.join(build) + '\n </build>\n</model>\n')
    ctypes = ('<?xml version="1.0" encoding="UTF-8"?>\n<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">\n'
              ' <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>\n'
              ' <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>\n'
              ' <Default Extension="config" ContentType="text/xml"/>\n</Types>\n')
    rels = ('<?xml version="1.0" encoding="UTF-8"?>\n<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">\n'
            ' <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>\n</Relationships>\n')
    msettings = '<?xml version="1.0" encoding="UTF-8"?>\n<config>\n' + '\n'.join(cfg) + '\n</config>\n'
    with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml', ctypes)
        z.writestr('_rels/.rels', rels)
        z.writestr('3D/3dmodel.model', model)
        z.writestr('Metadata/model_settings.config', msettings)

os.makedirs(f'{OUT}/stl_print_ready', exist_ok=True)
for part, (f, k, _) in PARTS.items():
    oriented(f, k).export(f'{OUT}/stl_print_ready/{f}')
for name, items in PLATES.items():
    check(name, items)
    write_3mf(f'{OUT}/{name}.3mf', items)
    print('ok', name, len(items), 'objects')
