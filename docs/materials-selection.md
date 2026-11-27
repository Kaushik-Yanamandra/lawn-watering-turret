# Materials selection

The turret lives outdoors in Houston: direct sun, summer surface temperatures that can pass 60 °C on dark plastic, heavy rain, humidity, and the odd freeze. Every material in it was chosen against those conditions. This page records why, and what is still unproven.

## Printed plastics

| Material | Glass transition (approx.) | Strengths | Weaknesses | Used for |
|---|---|---|---|---|
| **ASA** | ~100 °C | UV-stable (made for outdoor use), stiff, keeps its shape in sun | Warps and cracks between layers if it cools too fast while printing; fumes while printing | Parts in direct sun: P1 pan base, P2 yoke, P3 cradle, P4 pipe clamps, P5 mounting plates, P8 thermal visor |
| **PETG** | ~80 °C | Tough and slightly flexible, easy to print, good layer bonding | Less UV-resistant than ASA; creeps under constant load when hot | Shaded parts and parts that flex: P6 post clips, P7 sensor bracket, P9 thermal carrier |
| **PLA** | ~55–60 °C | Easy to print | Softens and sags in Texas sun; brittle | **Not used anywhere** |

The design requirement is to operate from 0 to 45 °C air temperature and survive 60 °C in sun. That rules out PLA and puts the most loaded sun-facing parts in ASA, which keeps a margin of about 40 °C below its glass transition.

**P6 post clips** sit on the post in sun but are PETG because a clip has to flex to snap on, and ASA is more likely to crack. They carry almost no load. Check them once a year for chalky, faded surfaces, the first sign of UV damage, and reprint in ASA if they start to crack.

## Designing for how 3D prints fail

A fused-filament print is not a uniform solid. It is strongest along its printed lines and weakest **between layers**, where each layer only partly fuses to the one below. Choices that follow from that:

- **Orientation per part.** Each part is printed with its largest flat face on the bed and its loads, where possible, running along the layers rather than pulling them apart. The P3 cradle halves are printed split-face down so the faces that clamp the brass nozzle adapter are flat and true. The full list is in the [print guide](../cad/print/bambu-p2s/README.md#why-each-part-sits-the-way-it-does).
- **A known compromise: the P2 yoke arms.** The yoke is printed base-down, so its two upright arms have layer lines running across the direction they bend. The loads are small (the jet's reaction is about 1.9 N and both axes pass through the nozzle, so the arms see little bending), and the part uses 4 walls and 60% infill for margin. This is the first part to inspect after the field trial.
- **Walls over infill.** Strength in bending and around holes comes mostly from the outer walls, so every part uses 4 walls; infill is 60% for the head parts and 40% elsewhere.
- **Heat-set inserts need surrounding material.** Brass inserts are pressed into the plastic with a hot tip and grip by the plastic re-forming around their knurls. The Rev A mounting plate (P5) used 7 mm inserts in a 5 mm plate, so the inserts would have broken through the back. Rev B adds an 11 mm boss behind each insert and makes the hole blind, with a 3 mm floor.

## Metals and corrosion

| Part | Material | Why | Watch for |
|---|---|---|---|
| Turret post | Galvanised steel pipe, ¾ in | Stiff and cheap; the zinc coating corrodes first and protects the steel | Cut ends and drilled holes lose their zinc: seal them with cold-galvanising paint |
| Fasteners | A2 stainless steel | Do not rust in rain | Stainless touching galvanised steel in standing water makes a galvanic pair in which the zinc corrodes faster. Plastic washers between them, or keeping the joint dry, reduce this. |
| Fittings and nozzle | Brass | Machinable, corrosion-resistant in water, standard for plumbing | Use lead-free brass for anything on the drinking-water side of the backflow preventer |
| Heat-set inserts | Brass | Grip well in thermoplastics and do not rust | — |

## Still to be proven

- How ASA and PETG parts actually age over a Houston summer: colour change, surface chalking and any loss of toughness.
- Whether the servos' plastic cases and seals tolerate sustained sun; a shade hood over the head is planned if they run hot.
- Creep of the PETG clips at summer temperatures.

These are part of the two-week field test and a longer check after the first summer.
