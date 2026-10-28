FeatureScript 3083;
import(path : "onshape/std/geometry.fs", version : "3083.0");

/*
 * HOIST design options: two lowering/rehanging end effectors and the SPINE head on three lifts.
 *
 *   HOIST Greenhouse        pipe rail, crop wires, one wire support per wire, winding hooks and plants
 *   HOIST EE-A Wire rider   rides the crop wire on a roller saddle; a normally closed twine capstan holds
 *                           the plant while a hook rotor on an X/Z slide flips and rehangs the hook
 *   HOIST EE-B Hook carrier grips the hook and carries the plant on the head; a load cell weighs it and
 *                           a pad on the wire is only a height datum
 *   HOIST SPINE S1/S2/S3    rail cart, lift (mast / scissor / telescopic), Y beam, Y/X head and a tool
 *
 * Frames: X across the path (+X toward the served crop wire), Y along the row, Z up, millimetres.
 * World origin on the floor at the pipe-rail centreline. Tool frame: origin at the crop-wire centre at
 * the gripped hook. Part names are "<group> | <part>"; a group is one rigid body in the MuJoCo demo
 * (design/hoist_demo.py).
 *
 * Every dimension is a first guess. Hook shape, twine store, free twine and plant are unmeasured.
 */

const MM = millimeter;

const WIRE_X = 400;                                   // crop wire 125 outboard of the pipe rail, unverified
const WIRE_Z = 3800;                                  // high-wire height, unverified
const ROW_Y = 2200;                                   // drawn row length, arbitrary
const HOOK_YS = [-1050, -700, -350, 0, 650];          // 350 pitch, unverified; +650 already moved, 0 is next
const SUPPORT_Y = -175;                               // one support between two hooks, unverified
const ZB_WORK = 3170;                                 // Y-beam underside at work: puts the tool mount 440 under the wire

function C() returns map
{
    return {
            "floor" : color(0.80, 0.80, 0.78),
            "steel" : color(0.58, 0.60, 0.63),
            "wire" : color(0.25, 0.25, 0.27),
            "galv" : color(0.70, 0.72, 0.74),
            "hook" : color(0.12, 0.45, 0.28),
            "twine" : color(0.95, 0.94, 0.88),
            "plant" : color(0.30, 0.58, 0.22),
            "alu" : color(0.80, 0.81, 0.83),
            "dark" : color(0.22, 0.23, 0.25),
            "rubber" : color(0.08, 0.08, 0.08),
            "A" : color(0.18, 0.42, 0.78),
            "A2" : color(0.40, 0.62, 0.92),
            "B" : color(0.86, 0.46, 0.14),
            "B2" : color(0.95, 0.66, 0.35),
            "sensor" : color(0.93, 0.76, 0.18),
            "cart" : color(0.30, 0.32, 0.35),
            "lift" : color(0.66, 0.68, 0.70),
            "scissor" : color(0.93, 0.72, 0.16),
            "head" : color(0.36, 0.48, 0.58),
            "head2" : color(0.20, 0.30, 0.40),
            "elec" : color(0.42, 0.52, 0.48)
        };
}

// ================================================================ features

annotation { "Feature Type Name" : "HOIST Greenhouse",
             "Feature Type Description" : "Pipe rail, crop wires, wire supports, winding hooks and plants around one SPINE path." }
export const hoistGreenhouse = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
    }
    {
        buildGreenhouse(context, new box({ "n" : 0, "base" : id + "gh" }));
    });

annotation { "Feature Type Name" : "HOIST EE-A Wire rider",
             "Feature Type Description" : "Saddle on the crop wire, normally closed twine capstan, hook rotor on an X/Z slide. Tool frame at the wire." }
export const hoistEeA = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Show wire, hook and plant", "Default" : true }
        definition.showContext is boolean;
    }
    {
        buildA(context, new box({ "n" : 0, "base" : id + "ee" }));
        if (definition.showContext)
            buildLocalContext(context, new box({ "n" : 0, "base" : id + "cx" }));
    });

annotation { "Feature Type Name" : "HOIST EE-B Hook carrier",
             "Feature Type Description" : "Hook rotor on a load cell, wire rest pad as datum; the plant hangs on the tool while the hook moves. Tool frame at the wire." }
export const hoistEeB = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Show wire, hook and plant", "Default" : true }
        definition.showContext is boolean;
    }
    {
        buildB(context, new box({ "n" : 0, "base" : id + "ee" }));
        if (definition.showContext)
            buildLocalContext(context, new box({ "n" : 0, "base" : id + "cx" }));
    });

annotation { "Feature Type Name" : "HOIST SPINE S1 Mast",
             "Feature Type Description" : "Rail cart with a fixed mast and a carriage that lifts the Y beam, head and tool." }
export const hoistS1 = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        systemParams(definition);
    }
    {
        buildSystem(context, id, definition, "S1");
    });

annotation { "Feature Type Name" : "HOIST SPINE S2 Scissor",
             "Feature Type Description" : "Rail cart with a four-stage scissor lift; risers carry the Y beam, head and tool." }
export const hoistS2 = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        systemParams(definition);
    }
    {
        buildSystem(context, id, definition, "S2");
    });

annotation { "Feature Type Name" : "HOIST SPINE S3 Telescopic",
             "Feature Type Description" : "Rail cart with a three-stage telescopic column carrying the Y beam, head and tool." }
export const hoistS3 = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        systemParams(definition);
    }
    {
        buildSystem(context, id, definition, "S3");
    });

predicate systemParams(definition is map)
{
    annotation { "Name" : "Lift up (tool docked on the hook at Y = 0)", "Default" : true }
    definition.liftUp is boolean;
    annotation { "Name" : "Tool A wire rider (off = tool B hook carrier)", "Default" : true }
    definition.toolA is boolean;
    annotation { "Name" : "Show greenhouse", "Default" : true }
    definition.showContext is boolean;
}

function buildSystem(context is Context, id is Id, definition is map, lift is string)
{
    const reg = new box({ "n" : 0, "base" : id + "sys" });
    const up = definition.liftUp;
    var zb = ZB_WORK;
    if (lift == "S1")
    {
        zb = up ? ZB_WORK : 1500;
        buildCart(context, reg, true);
        buildS1(context, reg, zb);
    }
    else if (lift == "S2")
    {
        const zp = up ? ZB_WORK - 340 : 680;
        zb = zp + 340;
        buildCart(context, reg, false);
        buildS2(context, reg, zp, zb);
    }
    else
    {
        zb = up ? ZB_WORK : 1395;
        buildCart(context, reg, true);
        buildS3(context, reg, zb);
    }
    const dz = zb - ZB_WORK;
    const xf = up ? 200 : 0;                 // X carriage front edge: 200 = docked, 0 = retracted
    buildHead(context, reg, dz, 0, xf);

    // The tool is drawn in its own frame, then carried to the X carriage.
    const tid = id + "tool";
    if (definition.toolA)
        buildA(context, new box({ "n" : 0, "base" : tid }));
    else
        buildB(context, new box({ "n" : 0, "base" : tid }));
    opTransform(context, id + "place", {
                "bodies" : qBodyType(qCreatedBy(tid, EntityType.BODY), BodyType.SOLID),
                "transform" : transform(v3(200 + xf, 0, WIRE_Z + dz)) });

    if (definition.showContext)
        buildGreenhouse(context, new box({ "n" : 0, "base" : id + "gh" }));
}

// ================================================================ greenhouse

function buildGreenhouse(context is Context, reg is box)
{
    const c = C();
    bx(context, reg, "floor", [-900, -ROW_Y, -20], [900, ROW_Y, 0]);
    mk(context, reg, "floor", "ctx | floor", c.floor);
    for (var s in [-1, 1])
    {
        const sn = s > 0 ? "+X" : "-X";
        const kr = "rail" ~ sn;
        cyl(context, reg, kr, [s * 275, -ROW_Y, 120], [s * 275, ROW_Y, 120], 25.5);
        for (var y = -2000; y <= 2000; y += 1000)
            bx(context, reg, kr, [s * 275 - 30, y - 15, 0], [s * 275 + 30, y + 15, 100]);
        mk(context, reg, kr, "ctx | pipe rail " ~ sn, c.steel);

        const kw = "wire" ~ sn;
        cyl(context, reg, kw, [s * WIRE_X, -ROW_Y, WIRE_Z], [s * WIRE_X, ROW_Y, WIRE_Z], 2.5);
        mk(context, reg, kw, "ctx | crop wire " ~ sn, c.wire);

        // Wire support: clamp around the wire, hanger rod up to the truss. Real ones are unmeasured.
        const ks = "support" ~ sn;
        bx(context, reg, ks, [s * WIRE_X - 10, SUPPORT_Y - 15, WIRE_Z - 12], [s * WIRE_X + 10, SUPPORT_Y + 15, WIRE_Z + 20]);
        cyl(context, reg, ks, [s * WIRE_X, SUPPORT_Y, WIRE_Z + 15], [s * WIRE_X, SUPPORT_Y, WIRE_Z + 600], 4);
        mk(context, reg, ks, "ctx | wire support " ~ sn, c.galv);
    }

    // One hook set drawn at the tool frame, then copied to every hook position (mirrored on -X).
    buildHookSet(context, reg, "proto ", 2250, ["proto hook", "proto store", "proto twine", "proto plant"]);
    var tfs = [];
    var names = [];
    for (var s in [-1, 1])
    {
        for (var y in HOOK_YS)
        {
            var t = transform(v3(s * WIRE_X, y, WIRE_Z));
            if (s < 0)
                t = t * mirrorAcross(plane(v3(0, 0, 0), vector(1, 0, 0), vector(0, 1, 0)));
            tfs = append(tfs, t);
            names = append(names, "h" ~ size(names));
        }
    }
    const parts = ["hook", "store", "twine", "plant"];
    const cols = [c.hook, c.twine, c.twine, c.plant];
    var protos = [];
    for (var i = 0; i < 4; i += 1)
    {
        const q = qKind(reg, "proto " ~ parts[i]);
        placeCopies(context, reg, q, tfs, names, "ctx | " ~ parts[i], cols[i]);
        protos = append(protos, q);
    }
    opDeleteBodies(context, nid(reg, "del"), { "entities" : qUnion(protos) });
}

// Wire stub, hook, twine and a short plant around the tool frame (EE Part Studios).
function buildLocalContext(context is Context, reg is box)
{
    cyl(context, reg, "wire", [0, -400, 0], [0, 400, 0], 2.5);
    mk(context, reg, "wire", "ctx | crop wire", C().wire);
    buildHookSet(context, reg, "", 500, ["hook | hook", "hook | twine store", "ctx | twine", "plant | plant"]);
}

// Stylised double-ended winding hook (3 mm wire S, shank 200, end arcs R7, tips 15) hanging on a wire
// along Y at the origin, with the twine wound lengthwise on the shank, the free twine and the plant.
// The hook plane is XZ; real hooks are unmeasured, including which way they flip.
function buildHookSet(context is Context, reg is box, kp is string, stemLen is number, names is array)
{
    const c = C();
    var pts = [[7, 0, -18]];
    for (var i = 0; i <= 6; i += 1)          // top cap over the wire, tip side to shank side
    {
        const a = (30 * i) * degree;
        pts = append(pts, [7 * cos(a), 0, -3 + 7 * sin(a)]);
    }
    for (var i = 0; i <= 6; i += 1)          // bottom cup, shank side to tip side
    {
        const a = (-30 * i) * degree;
        pts = append(pts, [-14 + 7 * cos(a), 0, -203 + 7 * sin(a)]);
    }
    pts = append(pts, [-21, 0, -188]);
    wire(context, reg, kp ~ "hook", pts, 1.5);
    mk(context, reg, kp ~ "hook", names[0], c.hook);

    bx(context, reg, kp ~ "store", [-13, -2.5, -183], [-1, 2.5, -23]);
    mk(context, reg, kp ~ "store", names[1], c.twine);

    cyl(context, reg, kp ~ "twine", [-14, 0, -211.5], [-14, 0, -515], 1);   // free twine, 300 (unmeasured)
    mk(context, reg, kp ~ "twine", names[2], c.twine);

    const kq = kp ~ "plant";
    const head = [-14, 0, -536.5];
    const u = normalize(vector(250, -400, -2200));                          // stem leans away from the path
    const e = [head[0] + u[0] * stemLen, head[1] + u[1] * stemLen, head[2] + u[2] * stemLen];
    ball(context, reg, kq, head, 25);
    cyl(context, reg, kq, head, e, 7);
    const dn = 25 * degree;
    const hs = [vector(1, 0, 0), normalize(vector(0.3, 1, 0)), normalize(vector(0.3, -1, 0))];
    const at = [70, 160, 250];
    for (var i = 0; i < 3; i += 1)
    {
        const h = hs[i];
        const b = [head[0] + u[0] * at[i], head[1] + u[1] * at[i], head[2] + u[2] * at[i]];
        leaf(context, reg, kq, b, vector(h[0] * cos(dn), h[1] * cos(dn), -sin(dn)), vector(-h[1], h[0], 0), 250, 80);
    }
    mk(context, reg, kq, names[3], c.plant);
}

// ================================================================ EE-A wire rider (tool frame)

function buildA(context is Context, reg is box)
{
    const c = C();
    bx(context, reg, "Am", [-300, -50, -440], [-200, 50, -430]);
    mk(context, reg, "Am", "A.frame | mount plate", c.dark);
    bx(context, reg, "Aw", [-300, -50, -430], [-200, 50, -420]);
    mk(context, reg, "Aw", "A.frame | compliant wrist", c.rubber);

    const kb = "Ab";
    bx(context, reg, kb, [-300, -100, -420], [-200, 100, -410]);                      // base plate
    plateX(context, reg, kb, -300, -290, rrect(-100, -420, 100, 80), 12);             // back plate
    plateY(context, reg, kb, -100, -90, rrect(-300, -200, -25, -30), 10);             // side plate, stays clear of supports
    plateZ(context, reg, kb, 62, 72, rrect(-300, -85, 25, 85), 12);                   // top bracket over the wire
    for (var yc in [-50, 50])                 // rollers at +-50: the hook fits between them 25 off-centre
    {                                         // while the fork slides on, and the rear one clears the support
        for (var xs in [-12, 9])
            bx(context, reg, kb, [xs, yc - 23, 10], [xs + 3, yc + 23, 63]);          // saddle roller forks
    }
    mk(context, reg, kb, "A.frame | body", c.alu);

    for (var yc in [-50, 50])                                                         // flanged saddle rollers on the wire
    {
        const kr = "Ar" ~ yc;
        cyl(context, reg, kr, [-7, yc, 22.5], [7, yc, 22.5], 20);
        cyl(context, reg, kr, [-9, yc, 22.5], [-7, yc, 22.5], 23);
        cyl(context, reg, kr, [7, yc, 22.5], [9, yc, 22.5], 23);
        cyl(context, reg, kr, [-12, yc, 22.5], [12, yc, 22.5], 3);
        mk(context, reg, kr, "A.frame | saddle roller", c.rubber);
    }

    bx(context, reg, "Axr", [-290, -90, -70], [-20, -82, -50]);
    mk(context, reg, "Axr", "A.frame | rotor X rail", c.steel);

    bx(context, reg, "Acam", [-130, 40, 32], [-100, 70, 62]);
    cyl(context, reg, "Acam", [-100, 55, 47], [-90, 55, 47], 7);
    mk(context, reg, "Acam", "A.frame | camera", c.rubber);

    bx(context, reg, "Asen", [-75, 30, -52], [-70, 50, 62]);
    cyl(context, reg, "Asen", [-70, 40, -40], [-30, 40, -40], 6);
    mk(context, reg, "Asen", "A.frame | inductive sensor", c.sensor);

    // Rotor X slide (130 back for flip clearance) and Z slide (30 up to unhook).
    bx(context, reg, "Arx", [-55, -82, -80], [-15, -68, -40]);
    bx(context, reg, "Arx", [-50, -68, -135], [-20, -64, -45]);
    mk(context, reg, "Arx", "A.rx | X carriage", c.A);

    // Z stage: rotor motor on top, capstan underneath, so hook and capstan move together.
    const kz = "Arz";
    bx(context, reg, kz, [-45, -64, -120], [-25, -52, -86]);
    bx(context, reg, kz, [-60, -64, -320], [-25, -52, -120]);                          // hanger down to the capstan
    plateZ(context, reg, kz, -320, -310, rrect(-140, -64, -40, 125), 8);              // capstan bracket
    bx(context, reg, kz, [-70, -32, -310], [-40, -2, -272]);                           // driven roller bearing block
    mk(context, reg, kz, "A.rz | Z stage", c.A);

    cyl(context, reg, "Arm", [-7, -52, -103], [-7, -28, -103], 20);
    mk(context, reg, "Arm", "A.rz | rotor motor", c.dark);

    cyl(context, reg, "Acr", [-40, -16.5, -290], [4, -16.5, -290], 15);
    mk(context, reg, "Acr", "A.rz | capstan roller", c.rubber);

    cyl(context, reg, "Acm", [-130, -16.5, -290], [-70, -16.5, -290], 14);
    bx(context, reg, "Acm", [-120, -26, -310], [-80, -7, -300]);
    mk(context, reg, "Acm", "A.rz | capstan motor", c.dark);

    cyl(context, reg, "Aic", [-55, 80, -290], [-55, 125, -290], 9);                  // spring-closed idler cylinder
    bx(context, reg, "Aic", [-65, 85, -310], [-45, 120, -296]);
    cyl(context, reg, "Aic#cut", [-55, 79, -290], [-55, 126, -290], 5);
    mk(context, reg, "Aic", "A.rz | idler cylinder", c.dark);

    // Idler: springs shut on the twine (normally closed). Opens 45 in +Y, out of the twine's way while the
    // fork slides on and off along the wire (the twine passes 25 off-centre).
    bx(context, reg, "Aid", [-70, 2, -309], [-40, 32, -272]);
    cyl(context, reg, "Aid", [-55, 32, -290], [-55, 80, -290], 4);
    mk(context, reg, "Aid", "A.idler | carrier", c.A2);
    cyl(context, reg, "Air", [-40, 16.5, -290], [4, 16.5, -290], 15);
    mk(context, reg, "Air", "A.idler | roller", c.rubber);

    buildYoke(context, reg, "A", 18, c.A, c.A2);
}

// ================================================================ EE-B hook carrier (tool frame)

function buildB(context is Context, reg is box)
{
    const c = C();
    bx(context, reg, "Bm", [-300, -50, -440], [-200, 50, -430]);
    mk(context, reg, "Bm", "B.frame | mount plate", c.dark);
    bx(context, reg, "Bw", [-300, -50, -430], [-200, 50, -420]);
    mk(context, reg, "Bw", "B.frame | compliant wrist", c.rubber);
    bx(context, reg, "Blc", [-280, -20, -420], [-230, 20, -390]);
    mk(context, reg, "Blc", "B.frame | load cell", c.sensor);

    const kb = "Bb";
    plateZ(context, reg, kb, -390, -370, rrect(-290, -110, -80, 40), 12);             // arm
    bx(context, reg, kb, [-110, -100, -370], [-80, -70, -80]);                         // post
    plateY(context, reg, kb, -80, -70, [[-110, -138], [-7, -138, 35], [-7, -68, 35], [-110, -68]], 0);
    mk(context, reg, kb, "B.frame | body", c.alu);

    cyl(context, reg, "Bd", [-7, -70, -103], [-7, -28, -103], 35);                    // braking gearbox
    cyl(context, reg, "Bd", [-7, -140, -103], [-7, -70, -103], 28);
    mk(context, reg, "Bd", "B.frame | rotor drive", c.dark);

    bx(context, reg, "Brb", [-24, -66, -80], [-12, -54, 20]);                         // wire rest: datum only
    mk(context, reg, "Brb", "B.frame | wire rest bar", c.alu);
    bx(context, reg, "Brp", [-12, -72, 2.5], [12, -48, 16]);
    mk(context, reg, "Brp", "B.frame | wire rest pad", c.rubber);

    bx(context, reg, "Bcam", [-110, -100, -80], [-80, -70, -50]);
    cyl(context, reg, "Bcam", [-80, -85, -65], [-70, -85, -65], 7);
    mk(context, reg, "Bcam", "B.frame | camera", c.rubber);

    bx(context, reg, "Bsen", [-80, 30, -370], [-75, 50, -34]);
    cyl(context, reg, "Bsen", [-75, 40, -40], [-35, 40, -40], 6);
    mk(context, reg, "Bsen", "B.frame | inductive sensor", c.sensor);

    buildYoke(context, reg, "B", 24, c.B, c.B2);
}

// Rotor output: side-entry fork at the hook centre (-7, -103). Hub plate on the -Y side; two rigid fingers
// reach +Y past the hook and close in X on the twine store (each opens 7.5). Motor axis along Y through
// the hook centre. The fork slides onto the hook along the wire (25) and off the same way.
// ⚠️ A U-yoke closed on its -X side traps the hook: after a half-flip the closed side is in front of the
// hook, so the tool can't back away (hoist_demo.py measured -9 mm). A hub-side fork keeps its opening
// toward +Y through any flip about Y.
function buildYoke(context is Context, reg is box, g is string, hubR is number, col is Color, jawCol is Color)
{
    const ky = g ~ "y";
    cyl(context, reg, ky, [-7, -28, -103], [-7, -22, -103], hubR);
    plateY(context, reg, ky, -24, -20, rrect(-42, -123, 20, -83), 6);
    bx(context, reg, ky, [-56, -24, -113], [-42, -10, -93]);                          // finger actuator
    mk(context, reg, ky, g ~ ".yoke | fork hub", col);
    plateX(context, reg, g ~ "jL", -23, -13, rrect(-20, -123, 12, -83), 5);
    mk(context, reg, g ~ "jL", g ~ ".jawL | back finger", jawCol);
    plateX(context, reg, g ~ "jR", -1, 9, rrect(-20, -123, 12, -83), 5);
    mk(context, reg, g ~ "jR", g ~ ".jawR | front finger", jawCol);
}

// ================================================================ SPINE cart, lifts, head (world)

function buildCart(context is Context, reg is box, elec is boolean)
{
    const c = C();
    plateZ(context, reg, "cc", 180, 330, rrect(-240, -550, 240, 550), 25);
    for (var sx in [-1, 1])
    {
        for (var sy in [-1, 1])
            cyl(context, reg, "cc", [sx * 230, sy * 420, 215.5], [sx * 255, sy * 420, 215.5], 12);
    }
    mk(context, reg, "cc", "base | chassis", c.cart);
    for (var sx in [-1, 1])
    {
        for (var sy in [-1, 1])
        {
            const kw = "cw" ~ sx ~ sy;
            cyl(context, reg, kw, [sx * 255, sy * 420, 215.5], [sx * 295, sy * 420, 215.5], 70);
            mk(context, reg, kw, "base | rail wheel", c.rubber);
        }
    }
    if (elec)
    {
        plateZ(context, reg, "ce", 330, 520, rrect(40, -200, 220, 300), 10);
        mk(context, reg, "ce", "base | electronics", c.elec);
    }
}

function buildS1(context is Context, reg is box, zb is number)
{
    const c = C();
    bx(context, reg, "s1m", [-120, -560, 330], [0, -440, 3330]);
    bx(context, reg, "s1m", [-160, -550, 330], [40, -390, 345]);
    mk(context, reg, "s1m", "S1.mast | mast", c.lift);

    const kt = "s1c";
    bx(context, reg, kt, [-140, -580, zb - 150], [-121, -420, zb + 60]);
    bx(context, reg, kt, [1, -580, zb - 150], [20, -420, zb + 60]);
    bx(context, reg, kt, [-140, -580, zb - 150], [20, -561, zb + 60]);
    bx(context, reg, kt, [-140, -439, zb - 150], [20, -420, zb + 60]);
    mk(context, reg, kt, "S1.top | carriage", c.head2);
    bx(context, reg, "s1b", [-100, -420, zb], [-20, 480, zb + 60]);
    mk(context, reg, "s1b", "S1.top | Y beam", c.lift);
}

function buildS2(context is Context, reg is box, zp is number, zb is number)
{
    const c = C();
    bx(context, reg, "s2f", [-250, -550, 330], [250, 550, 380]);
    mk(context, reg, "s2f", "S2.base | frame", c.lift);

    const L = 1000;
    const z0 = 410;
    const h = (zp - 30 - z0) / 4;
    const span = L * cos(asin(h / L));
    const y0 = -500;                          // fixed pins; the +Y ends roll
    for (var i = 0; i < 4; i += 1)
    {
        const za = z0 + i * h;
        for (var sx in [-1, 1])
        {
            arm(context, reg, "s2a" ~ i ~ sx, sx * 180, [y0, za], [y0 + span, za + h]);
            mk(context, reg, "s2a" ~ i ~ sx, "S2.arm | stage " ~ (i + 1), c.scissor);
            arm(context, reg, "s2b" ~ i ~ sx, sx * 210, [y0 + span, za], [y0, za + h]);
            mk(context, reg, "s2b" ~ i ~ sx, "S2.arm | stage " ~ (i + 1), c.scissor);
        }
        cyl(context, reg, "s2c" ~ i, [-222, y0 + span / 2, za + h / 2], [222, y0 + span / 2, za + h / 2], 10);
        mk(context, reg, "s2c" ~ i, "S2.pin | centre pin", c.steel);
    }
    for (var i = 0; i <= 4; i += 1)
    {
        for (var e = 0; e < 2; e += 1)
        {
            const kp = "s2p" ~ i ~ e;
            cyl(context, reg, kp, [-222, y0 + e * span, z0 + i * h], [222, y0 + e * span, z0 + i * h], 10);
            mk(context, reg, kp, "S2.pin | end pin", c.steel);
        }
    }
    const kt = "s2t";
    bx(context, reg, kt, [-250, -550, zp], [250, 550, zp + 40]);
    for (var sy in [-1, 1])
        bx(context, reg, kt, [-100, sy * 400 - 20, zp + 40], [-20, sy * 400 + 20, zb]);
    bx(context, reg, kt, [-100, -450, zb], [-20, 450, zb + 60]);
    mk(context, reg, kt, "S2.top | platform and Y beam", c.lift);
}

// Scissor arm: 50 wide slot between two pin centres in the YZ plane, 20 thick, centred on x = xc.
function arm(context is Context, reg is box, k is string, xc is number, p is array, q is array)
{
    const d = normalize(vector(q[0] - p[0], q[1] - p[1]));
    const n = vector(-d[1], d[0]);
    const a = vector(p[0], p[1]) - d * 25;
    const b = vector(q[0], q[1]) + d * 25;
    const pts = [[a[0] - n[0] * 25, a[1] - n[1] * 25], [b[0] - n[0] * 25, b[1] - n[1] * 25],
                 [b[0] + n[0] * 25, b[1] + n[1] * 25], [a[0] + n[0] * 25, a[1] + n[1] * 25]];
    plateX(context, reg, k, xc - 10, xc + 10, pts, 25);
}

function buildS3(context is Context, reg is box, zb is number)
{
    const c = C();
    const ext = (zb - 20 - 1375) / 3;         // each stage slides out the same amount
    sqTube(context, reg, "s3b", 160, 330, 1330);
    mk(context, reg, "s3b", "S3.base | sleeve", c.lift);
    const sides = [140, 120, 100];
    const bottoms = [345, 360, 375];
    for (var i = 0; i < 3; i += 1)
    {
        const k = "s3s" ~ i;
        const z0 = bottoms[i] + (i + 1) * ext;
        sqTube(context, reg, k, sides[i], z0, z0 + 1000);
        mk(context, reg, k, "S3.s" ~ (i + 1) ~ " | stage", i == 1 ? c.head : c.lift);
    }
    bx(context, reg, "s3t", [-140, -580, zb - 20], [20, -380, zb]);
    bx(context, reg, "s3t", [-100, -420, zb], [-20, 480, zb + 60]);
    mk(context, reg, "s3t", "S3.top | top plate and Y beam", c.head2);
}

// Square tube of 6 mm walls centred at (x -60, y -500).
function sqTube(context is Context, reg is box, k is string, side is number, z0 is number, z1 is number)
{
    const x0 = -60 - side / 2;
    const x1 = -60 + side / 2;
    const y0 = -500 - side / 2;
    const y1 = -500 + side / 2;
    bx(context, reg, k, [x0, y0, z0], [x0 + 6, y1, z1]);
    bx(context, reg, k, [x1 - 6, y0, z0], [x1, y1, z1]);
    bx(context, reg, k, [x0, y0, z0], [x1, y0 + 6, z1]);
    bx(context, reg, k, [x0, y1 - 6, z0], [x1, y1, z1]);
}

// Y carriage on the beam with the X rail on top; X carriage carries the tool mount at Z 3360 (at work).
function buildHead(context is Context, reg is box, dz is number, yc is number, xf is number)
{
    const c = C();
    bx(context, reg, "hy", [-110, yc - 70, 3230 + dz], [-10, yc + 70, 3290 + dz]);
    mk(context, reg, "hy", "head.y | Y carriage", c.head);
    bx(context, reg, "hr", [-100, yc - 25, 3290 + dz], [200, yc + 25, 3320 + dz]);
    mk(context, reg, "hr", "head.y | X rail", c.steel);
    bx(context, reg, "hx", [xf - 100, yc - 50, 3320 + dz], [xf, yc + 50, 3360 + dz]);
    mk(context, reg, "hx", "head.x | X carriage", c.head2);
}

// ================================================================ part builder

// Unions everything built under kind k (minus kind k#cut) into one part, then names and colours it.
function mk(context is Context, reg is box, k is string, name is string, col is Color)
{
    const q = qKind(reg, k);
    if (size(evaluateQuery(context, q)) > 1)
    {
        try silent
        {
            opBoolean(context, nid(reg, "ops"), { "tools" : q, "operationType" : BooleanOperationType.UNION });
        }
    }
    const cuts = qKind(reg, k ~ "#cut");
    if (size(evaluateQuery(context, cuts)) > 0)
        opBoolean(context, nid(reg, "ops"), { "tools" : cuts, "targets" : qKind(reg, k), "operationType" : BooleanOperationType.SUBTRACTION });
    nameAs(context, qKind(reg, k), name, col);
}

function bx(context is Context, reg is box, k is string, p is array, q is array)
{
    box3(context, nid(reg, k), p, q);
}

function cyl(context is Context, reg is box, k is string, p is array, q is array, r is number)
{
    cylAxis(context, nid(reg, k), p, q, r);
}

function ball(context is Context, reg is box, k is string, p is array, r is number)
{
    opSphere(context, nid(reg, k), { "center" : v3(p[0], p[1], p[2]), "radius" : r * MM });
}

// Bent wire: cylinders between the points, a sphere at every point.
function wire(context is Context, reg is box, k is string, pts is array, r is number)
{
    for (var i = 0; i < size(pts); i += 1)
    {
        ball(context, reg, k, pts[i], r);
        if (i > 0)
            cyl(context, reg, k, pts[i - 1], pts[i], r);
    }
}

// Rounded plate in the YZ plane, outline [y, z] (optional third value = corner radius), from x0 to x1.
function plateX(context is Context, reg is box, k is string, x0 is number, x1 is number, pts is array, r is number)
{
    prismPlaneR(context, nid(reg, k), plane(v3(x0, 0, 0), vector(1, 0, 0), vector(0, 1, 0)), pts, r, x1 - x0);
}

// Rounded plate in the XZ plane, outline [x, z], from y0 to y1. Sketch axes on this plane are (Z, X).
function plateY(context is Context, reg is box, k is string, y0 is number, y1 is number, pts is array, r is number)
{
    var sw = [];
    for (var p in pts)
        sw = append(sw, size(p) > 2 ? [p[1], p[0], p[2]] : [p[1], p[0]]);
    prismPlaneR(context, nid(reg, k), plane(v3(0, y0, 0), vector(0, 1, 0), vector(0, 0, 1)), sw, r, y1 - y0);
}

// Rounded plate in the XY plane, outline [x, y], from z0 to z1.
function plateZ(context is Context, reg is box, k is string, z0 is number, z1 is number, pts is array, r is number)
{
    prismPlaneR(context, nid(reg, k), plane(v3(0, 0, z0), vector(0, 0, 1), vector(1, 0, 0)), pts, r, z1 - z0);
}

// Rounded diamond leaf, 2 thick, from base along dir; side lies in the leaf plane.
function leaf(context is Context, reg is box, k is string, base is array, dir is Vector, side is Vector, len is number, w is number)
{
    const pl = plane(v3(base[0], base[1], base[2]), normalize(cross(dir, side)), dir);
    prismPlaneR(context, nid(reg, k), pl, [[0, 0], [len * 0.45, w / 2], [len, 0], [len * 0.45, -w / 2]], 12, 2);
}

function placeCopies(context is Context, reg is box, part is Query, tfs is array, names is array, label is string, col is Color)
{
    const pid = nid(reg, "asm");
    opPattern(context, pid, { "entities" : part, "transforms" : tfs, "instanceNames" : names });
    nameAs(context, qCreatedBy(pid, EntityType.BODY), label, col);
}

function nameAs(context is Context, q is Query, name is string, col is Color)
{
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.NAME, "value" : name });
    setProperty(context, { "entities" : q, "propertyType" : PropertyType.APPEARANCE, "value" : col });
}

// ================================================================ geometry helpers (from the SO-101 greenhouse)

function v3(x is number, y is number, z is number) returns Vector
{
    return vector(x, y, z) * MM;
}

function nid(reg is box, kind is string) returns Id
{
    var r = reg[];
    r.n = r.n + 1;
    const newId = r.base + ("o" ~ r.n);
    var lst = r[kind];
    if (lst == undefined)
        lst = [];
    r[kind] = append(lst, newId);
    reg[] = r;
    return newId;
}

function qKind(reg is box, kind is string) returns Query
{
    var qs = [];
    const lst = reg[][kind];
    if (lst != undefined)
    {
        for (var i in lst)
            qs = append(qs, qCreatedBy(i, EntityType.BODY));
    }
    return qBodyType(qUnion(qs), BodyType.SOLID);
}

function box3(context is Context, id is Id, p is array, q is array)
{
    fCuboid(context, id, {
                "corner1" : v3(min(p[0], q[0]), min(p[1], q[1]), min(p[2], q[2])),
                "corner2" : v3(max(p[0], q[0]), max(p[1], q[1]), max(p[2], q[2])) });
}

function cylAxis(context is Context, id is Id, p is array, q is array, r is number)
{
    fCylinder(context, id, { "bottomCenter" : v3(p[0], p[1], p[2]), "topCenter" : v3(q[0], q[1], q[2]), "radius" : r * MM });
}

function rrect(x0 is number, y0 is number, x1 is number, y1 is number) returns array
{
    return [[min(x0, x1), min(y0, y1)], [max(x0, x1), min(y0, y1)], [max(x0, x1), max(y0, y1)], [min(x0, x1), max(y0, y1)]];
}

function prismPlaneR(context is Context, id is Id, pl is Plane, pts is array, r is number, depth is number)
{
    const skId = id + "sk";
    const sk = newSketchOnPlane(context, skId, { "sketchPlane" : pl });
    sketchRounded(sk, pts, r);
    skSolve(sk);
    opExtrude(context, id + "ex", {
                "entities" : qSketchRegion(skId),
                "direction" : pl.normal,
                "endBound" : BoundingType.BLIND,
                "endDepth" : depth * MM });
    opDeleteBodies(context, id + "del", { "entities" : qCreatedBy(skId, EntityType.BODY) });
}

function sketchRounded(sk is Sketch, pts is array, r is number)
{
    const n = size(pts);
    var t1 = [];
    var t2 = [];
    var mid = [];
    var has = [];
    for (var i = 0; i < n; i += 1)
    {
        const ip = (i + n - 1) % n;
        const inx = (i + 1) % n;
        const p = vector(pts[i][0], pts[i][1]);
        const pp = vector(pts[ip][0], pts[ip][1]);
        const pn = vector(pts[inx][0], pts[inx][1]);
        const ri = size(pts[i]) > 2 ? pts[i][2] : r;
        const ua = normalize(pp - p);
        const ub = normalize(pn - p);
        const phi = acos(max(-1, min(1, dot(ua, ub))));
        var t = 0;
        var rr = ri;
        if (ri > 0 && phi < 178 * degree && phi > 2 * degree)
        {
            t = ri / tan(phi / 2);
            const tMax = 0.49 * min(norm(pp - p), norm(pn - p));
            if (t > tMax)
            {
                t = tMax;
                rr = t * tan(phi / 2);
            }
        }
        if (t > 0.005)
        {
            const m = normalize(ua + ub);
            const ctr = p + m * (rr / sin(phi / 2));
            t1 = append(t1, p + ua * t);
            t2 = append(t2, p + ub * t);
            mid = append(mid, ctr - m * rr);
            has = append(has, true);
        }
        else
        {
            t1 = append(t1, p);
            t2 = append(t2, p);
            mid = append(mid, p);
            has = append(has, false);
        }
    }
    for (var i = 0; i < n; i += 1)
    {
        const j = (i + 1) % n;
        if (norm(t1[j] - t2[i]) > 0.005)
            skLineSegment(sk, "l" ~ i, { "start" : t2[i] * MM, "end" : t1[j] * MM });
        if (has[i])
            skArc(sk, "a" ~ i, { "start" : t1[i] * MM, "mid" : mid[i] * MM, "end" : t2[i] * MM });
    }
}

