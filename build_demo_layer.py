"""Build chem_lab_demo.usda: chem_lab_scene.usda + a demo "lab accident" on top.

  * physics beakers clustered on the bench module ahead-right of the robot
  * one "hazard" beaker filled with PhysX particle fluid (toxic green) that slides off the
    front edge of the bench onto the open floor in front of the robot when Play is pressed

chem_lab_scene.usda is NOT modified (it stays a sublayer, so its ROS graph is intact for the
Isaac 6.0.1 server). Needs Isaac Sim's python (PhysxSchema for the particle fluid). Re-runnable.

    ~/miniconda3/envs/env_isaacsim/bin/python build_demo_layer.py          # build
    ~/miniconda3/envs/env_isaacsim/bin/python build_demo_layer.py --test   # build + headless 4 s sim check
"""
import math
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from omni.physx.scripts import particleUtils  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt  # noqa: E402

ROOT = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(ROOT, "chem_lab_scene.usda")
DEMO = os.path.join(ROOT, "chem_lab_demo.usda")

BEAKER_SRC = "/root/Lab/ST_Acc_BechersGlass03_mo"   # 6.8 cm dia x 7.55 cm, pivot at bottom centre, axis +Z
BEAKER_MESH = "ST_Acc_BechersGlass03_md"
BEAKER_GLASS = "/ChemLab/materials/ST_Acc_ScienceGlass01"
BEAKER_GRIP = "/PhysicsMaterials/BeakerGrip"
BEAKER_R, BEAKER_H = 0.0341, 0.0755
BEAKER_MASS = 0.05
BEAKER_SDF_RES = 300
PHYSICS_SCENE = "/physicsScene"

DESK_TOP = -1.902               # flat bench top z (ray-cast of ST_Furn_ScienceDesk01_mo_001 mesh)
# Bench module R (+X side of the aisle, ahead-right of the robot): top spans x in [-5.26, -2.87], y in [3.92, 4.62].
# A sink (z -2.06) occupies x in [-5.08, -4.86], y in [4.10, 4.42]; flat areas are the aisle-side strip
# x in [-5.26, -5.14] and the front band y in [3.92, 4.03]. The FireBanket hangs on the x = -5.27 side,
# so the drop is over the FRONT edge (y = 3.91) into the open floor where the robot stands (base_link y = 3.15).
FRONT_EDGE_Y = 3.91

# "titration in progress" on the flat right half of the module (x in [-4.78, -2.90], y in [3.92, 4.62]):
# burette stand behind the hazard beaker, beakers/flasks/reagent bottles/balance around it.
# upright physics beakers (world x, y); >= 10 cm centre spacing (dia 6.8 cm)
DESK_BEAKERS = {
    "DeskBeaker_1": (-4.32, 3.99),
    "DeskBeaker_2": (-4.22, 4.10),
    "DeskBeaker_3": (-4.10, 3.98),
    "DeskBeaker_4": (-3.45, 4.35),   # on the balance pan (ElecBalance01, pan top +0.073)
    "DeskBeaker_5": (-3.30, 3.99),
}
ON_BALANCE = {"DeskBeaker_4": 0.073}
# static props referenced from lab.usd: name -> (source prim, world x, y, yaw deg, collider)
PROPS = {
    "Burette":       ("/root/Lab/ST_Acc_Buret01_mo",           -5.19, 4.10,  90.0, False),   # aisle strip, next to the hazard beaker
    "FlaskTall":     ("/root/Lab/ST_Acc_ErlenmeyerGlass02_mo", -3.92, 4.15,  20.0, False),
    "FlaskWide":     ("/root/Lab/ST_Acc_ErlenmeyerGlass01_mo", -3.75, 3.99, -30.0, False),
    "Reagent1":      ("/root/Lab/ST_Acc_ChemistryBottle01_mo", -4.30, 4.45, 120.0, False),
    "Reagent2":      ("/root/Lab/ST_Acc_ChemistryBottle01_mo", -4.13, 4.48,  80.0, False),
    "Balance":       ("/root/Lab/ST_Acc_ElecBalance01_mo",     -3.45, 4.35,   5.0, True),
    "Burner":        ("/root/Lab/ST_Acc_BunsenBurner01_mo",    -3.65, 4.42,   0.0, False),
    "Sponge":        ("/root/Lab/ST_Acc_Sponge01_mo",          -5.20, 4.50, -20.0, False),
}
DESK_SDF_RES = 150                          # decorative ones cook faster; hazard keeps 300 for the grasp

# hazard beaker: on the front edge (rim tangent to the drop), pushed toward the aisle (-Y)
HAZARD_XY = (-5.10, FRONT_EDGE_Y + 0.035)   # front-left corner by the aisle: easy reach for the robot
HAZARD_VEL = (0.0, -1.0, 0.0)               # m/s at t=0 (beaker AND fluid); mu=1.0 slide length v^2/(2*mu*g) = 5.7 cm > 3.5 cm to the edge; 1.5 m/s threw it 1.5 m, behind the robot

# fluid: the 1.3 mm glass wall leaks PBD particles, so an invisible thicker cup (purpose=guide) lines the inside
CUP_R_IN, CUP_WALL, CUP_SEGS = 0.0268, 0.006, 16     # wall boxes span r in [0.0268, 0.0328] (glass inner r ~0.0328)
CUP_FLOOR_Z = (0.002, 0.008)                        # bottom disc z range, beaker-local
CUP_WALL_Z = (0.008, 0.078)
PARTICLE_CONTACT_OFFSET = 0.003                     # m -> fluidRestOffset 0.00178, spacing 3.6 mm (~1700 particles)
FLUID_R, FLUID_Z = 0.022, (0.010, 0.050)            # fill radius / height range, beaker-local (~4 cm); keep > contactOffset clear of the cup wall
FLUID_COLOR = (0.15, 1.0, 0.05)

# decorative liquids in the other beakers (OmniGlass colour, fill height); visual only, no physics
DESK_LIQUIDS = {
    "DeskBeaker_1": ((0.10, 0.35, 1.00), 0.045),    # blue
    "DeskBeaker_2": ((1.00, 0.10, 0.25), 0.030),    # red
    "DeskBeaker_3": ((1.00, 0.85, 0.05), 0.050),    # yellow
    "DeskBeaker_4": ((0.05, 0.90, 0.90), 0.035),    # cyan
    "DeskBeaker_5": ((0.65, 0.10, 0.95), 0.040),    # purple
}
LIQUID_R = 0.030                                     # inside the 1.3 mm glass wall

# --- emergency timeline (seconds; the beaker leaves the bench at ~0.5 s and lands at ~0.8 s) ---
FPS = 60
T_SPILL_GAS = 1.0                # toxic fumes start rising from the puddle
T_SIREN = 2.0                    # beacons flash, room lights drop to emergency level
T_ROOM_GAS = 2.0                 # wall vents + room haze start (with the siren)
LANDING = (-5.10, 3.55, -2.75)
CAMERA_POS = (-9.6533, 1.0845, 0.6561)      # viewport start view, same as /Cameras/LabView in chem_lab_scene.usda
CAMERA_TARGET = (-4.856, 4.7249, -3.8533)   # hazard beaker rest position; headless --test runs land within ~10 cm of this

# Flow gas (room is roughly x [-10.9, 0], y [-7, 11.3], z [-2.78, 1.4]; stage units are metres)
GAS_CELL = 0.10                  # m, density cell size; ~1.6 M cells for the whole room, cheap for Flow
GAS_COLOR = (0.25, 1.0, 0.08)
ROOM_FILL = ((-5.5, 2.2, -1.6), (5.3, 9.0, 1.1))   # whole-room box, weak: raises the haze everywhere
ROOM_VENTS = {                   # thin boxes along the walls at floor level: (centre, halfSize)
    "VentWest":  ((-10.6, 2.0, -2.6), (0.15, 6.0, 0.12)),
    "VentEast":  ((-0.4, 2.0, -2.6), (0.15, 6.0, 0.12)),
    "VentSouth": ((-5.5, -6.6, -2.6), (5.0, 0.15, 0.12)),
    "VentNorth": ((-5.5, 10.9, -2.6), (5.0, 0.15, 0.12)),
}

# siren: the lab goes dark (dim, bluish) and two red strobes on the ceiling flash alternately
BEACONS = {"StrobeAisle": ((-6.2, 3.0, 1.25), 0.0), "StrobeBench": ((-3.8, 4.8, 1.25), 0.0)}   # pos, phase (fraction of period); in sync -> dark/red blink
BEACON_DUTY = 0.3                # fraction of each period the strobes are on (short, hard flashes)
BEACON_INTENSITY = 4000000.0            # radius 0.15 -> ~10x the power of a lab fill light (30000 @ r 0.5)
BEACON_HZ = 1.2
LAB_LIGHTS = {                   # path: (emergency intensity factor, emergency colour)
    "/ChemLab/Point_LabCeiling/Point_002": (0.08, (0.7, 0.8, 1.0)),
    "/ChemLab/______/______": (0.06, (0.7, 0.8, 1.0)),
    "/ChemLab/Point_LabFill1/Point_001": (0.10, (0.7, 0.8, 1.0)),
    "/ChemLab/Point_LabFill2/Point": (0.10, (0.7, 0.8, 1.0)),
    "/Environment/defaultLight": (0.10, (0.8, 0.85, 1.0)),
}


def make_beaker(st, path, xy, z, rot=(0.0, 0.0, 0.0), mass=BEAKER_MASS, sdf_res=BEAKER_SDF_RES):
    prim = st.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference("./chem_lab/lab.usd", BEAKER_SRC)
    # the source prim's xformOpOrder is translate/rotateXYZ/scale; override the values only
    prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(xy[0], xy[1], z))
    prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(*rot))
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)

    mesh = st.GetPrimAtPath(f"{path}/{BEAKER_MESH}")
    UsdPhysics.CollisionAPI.Apply(mesh)
    UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr("sdf")
    mesh.AddAppliedSchema("PhysxSDFMeshCollisionAPI")
    mesh.CreateAttribute("physxSDFMeshCollision:sdfResolution", Sdf.ValueTypeNames.Int, custom=False).Set(sdf_res)
    binding = UsdShade.MaterialBindingAPI.Apply(mesh)
    binding.Bind(UsdShade.Material(st.GetPrimAtPath(BEAKER_GLASS)))
    binding.Bind(UsdShade.Material(st.GetPrimAtPath(BEAKER_GRIP)), UsdShade.Tokens.weakerThanDescendants, "physics")
    return prim


def make_cup(st, beaker_path):
    """Invisible convex collision lining inside the beaker so fluid particles cannot tunnel through the glass."""
    cup = st.DefinePrim(f"{beaker_path}/Cup", "Xform")
    floor = UsdGeom.Cylinder.Define(st, f"{beaker_path}/Cup/Floor")
    floor.CreateRadiusAttr(CUP_R_IN + CUP_WALL)
    floor.CreateHeightAttr(CUP_FLOOR_Z[1] - CUP_FLOOR_Z[0])
    floor.CreateAxisAttr(UsdGeom.Tokens.z)
    floor.AddTranslateOp().Set(Gf.Vec3d(0, 0, sum(CUP_FLOOR_Z) / 2))
    floor.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    r_mid = CUP_R_IN + CUP_WALL / 2
    seg_w = 2 * math.pi * (CUP_R_IN + CUP_WALL) / CUP_SEGS * 1.05    # slight overlap, no gaps
    for i in range(CUP_SEGS):
        a = 2 * math.pi * i / CUP_SEGS
        box = UsdGeom.Cube.Define(st, f"{beaker_path}/Cup/Wall_{i:02d}")
        box.CreateSizeAttr(1.0)
        box.AddTranslateOp().Set(Gf.Vec3d(r_mid * math.cos(a), r_mid * math.sin(a), sum(CUP_WALL_Z) / 2))
        box.AddRotateZOp().Set(math.degrees(a))
        box.AddScaleOp().Set(Gf.Vec3f(CUP_WALL, seg_w, CUP_WALL_Z[1] - CUP_WALL_Z[0]))
        box.CreatePurposeAttr(UsdGeom.Tokens.guide)
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    return cup


def make_fluid_material(st, path):
    """OmniGlass render material (renders translucent in both RTX modes) that also carries the PBD fluid params."""
    mat = UsdShade.Material.Define(st, path)
    sh = UsdShade.Shader.Define(st, path + "/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(Sdf.AssetPath("OmniGlass.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniGlass", "mdl")
    sh.CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*FLUID_COLOR))
    sh.CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(1.33)
    sh.CreateInput("depth", Sdf.ValueTypeNames.Float).Set(0.02)
    sh.CreateInput("frosting_roughness", Sdf.ValueTypeNames.Float).Set(0.05)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateDisplacementOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    mat.CreateVolumeOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    # sticky-ish liquid: with water-like values the 1.7k particles scatter into separate blobs on impact
    particleUtils.add_pbd_particle_material(st, path, friction=0.2, viscosity=8.0, cohesion=0.25,
                                            surface_tension=0.15, damping=0.5, density=1000.0)
    return mat


def make_fluid(st, beaker_xy, beaker_z, v0):
    fx = st.DefinePrim("/DemoFX", "Xform")
    sys_path = Sdf.Path("/DemoFX/FluidSystem")
    particleUtils.add_physx_particle_system(
        st, sys_path, simulation_owner=Sdf.Path(PHYSICS_SCENE),
        particle_contact_offset=PARTICLE_CONTACT_OFFSET, enable_ccd=True,
        solver_position_iterations=8, max_velocity=6.0)
    particleUtils.add_physx_particle_anisotropy(st, sys_path, enabled=True, scale=5.0, min=1.0, max=2.0)
    particleUtils.add_physx_particle_smoothing(st, sys_path, enabled=True, strength=0.8)
    fluid_rest = 0.99 * 0.6 * PARTICLE_CONTACT_OFFSET
    particleUtils.add_physx_particle_isosurface(
        st, sys_path, enabled=True, grid_spacing=1.2 * fluid_rest, surface_distance=4.0 * fluid_rest,
        grid_filtering_passes="GSRS", grid_smoothing_radius=3.0 * fluid_rest,
        num_mesh_smoothing_passes=6, num_mesh_normal_smoothing_passes=6)
    mat = make_fluid_material(st, "/Looks/HazardFluid")
    b = UsdShade.MaterialBindingAPI.Apply(st.GetPrimAtPath(sys_path))
    b.Bind(mat)
    b.Bind(mat, materialPurpose="physics")

    # particle grid clipped to a cylinder, in world coordinates (the set must NOT be a child of the rigid body)
    spacing = 2.0 * fluid_rest
    n = int(2 * FLUID_R / spacing) + 1
    nz = int((FLUID_Z[1] - FLUID_Z[0]) / spacing) + 1
    pos = []
    for k in range(nz):
        for j in range(n):
            for i in range(n):
                x, y = -FLUID_R + i * spacing, -FLUID_R + j * spacing
                if x * x + y * y <= FLUID_R * FLUID_R:
                    pos.append(Gf.Vec3f(beaker_xy[0] + x, beaker_xy[1] + y, beaker_z + FLUID_Z[0] + k * spacing))
    particleUtils.add_physx_particleset_points(
        st, Sdf.Path("/DemoFX/HazardFluid"), Vt.Vec3fArray(pos), Vt.Vec3fArray([Gf.Vec3f(*v0)] * len(pos)),   # same push as the beaker, else its inertia stops the slide
        Vt.FloatArray([2 * fluid_rest] * len(pos)), sys_path,
        self_collision=True, fluid=True, particle_group=0, particle_mass=0.0, density=1000.0)
    pts = st.GetPrimAtPath("/DemoFX/HazardFluid")
    UsdShade.MaterialBindingAPI.Apply(pts).Bind(mat)
    UsdGeom.Imageable(pts).CreateVisibilityAttr(UsdGeom.Tokens.invisible)   # render the isosurface, not the raw points
    return len(pos)


def make_glass_material(st, path, color):
    mat = UsdShade.Material.Define(st, path)
    sh = UsdShade.Shader.Define(st, path + "/Shader")
    sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
    sh.SetSourceAsset(Sdf.AssetPath("OmniGlass.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier("OmniGlass", "mdl")
    sh.CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(1.33)
    sh.CreateInput("depth", Sdf.ValueTypeNames.Float).Set(0.02)
    for out in ("surface", "displacement", "volume"):
        getattr(mat, f"Create{out.capitalize()}Output")("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
    return mat


def make_desk_liquids(st):
    for name, (color, h) in DESK_LIQUIDS.items():
        cyl = UsdGeom.Cylinder.Define(st, f"/DemoBeakers/{name}/Liquid")
        cyl.CreateRadiusAttr(LIQUID_R)
        cyl.CreateHeightAttr(h)
        cyl.CreateAxisAttr(UsdGeom.Tokens.z)
        cyl.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.002 + h / 2))
        mat = make_glass_material(st, f"/Looks/Liquid_{name}", color)
        UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(mat)


def make_props(st):
    st.DefinePrim("/DemoProps", "Xform")
    z = DESK_TOP + 0.001
    for name, (src, x, y, yaw, collider) in PROPS.items():
        prim = st.DefinePrim(f"/DemoProps/{name}", "Xform")
        prim.GetReferences().AddReference("./chem_lab/lab.usd", src)
        prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, y, z))
        prim.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(0, 0, yaw))
        if collider:                      # e.g. the balance: a beaker stands on its pan
            for mesh in prim.GetChildren():
                UsdPhysics.CollisionAPI.Apply(mesh)
                UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr("convexHull")


def frame(t):
    return Usd.TimeCode(round(t * FPS))


def set_switch(attr, before, after, t):
    """Held value `before` until time t (s), then `after` (two samples one frame apart -> no visible ramp)."""
    attr.Set(before, Usd.TimeCode(0))
    attr.Set(before, Usd.TimeCode(round(t * FPS) - 1))
    attr.Set(after, frame(t))


def flow_attr(prim, name, typ, value, time=None):
    a = prim.CreateAttribute(name, typ, custom=False)
    a.Set(value) if time is None else a.Set(value, time)
    return a


def make_gas(st):
    """Omni Flow smoke: fumes from the puddle, then the wall vents fill the room. Needs omni.flowusd enabled."""
    T = Sdf.ValueTypeNames
    gas = st.DefinePrim("/DemoFX/Gas", "Xform")

    sim = st.DefinePrim("/DemoFX/Gas/flowSimulate", "FlowSimulate")
    flow_attr(sim, "layer", T.Int, 0)
    flow_attr(sim, "densityCellSize", T.Float, GAS_CELL)
    flow_attr(sim, "stepsPerSecond", T.Float, 30.0)
    adv = st.DefinePrim("/DemoFX/Gas/flowSimulate/advection", "FlowAdvectionCombustionParams")
    flow_attr(adv, "combustionEnabled", T.Bool, False)
    flow_attr(adv, "gravity", T.Float3, Gf.Vec3f(0, 0, -1.0))
    flow_attr(adv, "buoyancyPerTemp", T.Float, 0.3)     # heavy gas: barely rises, lingers at floor/camera height
    flow_attr(adv, "buoyancyMaxSmoke", T.Float, 1.0)
    # colormap x = temperature (schema doc), so temperature must NOT fade or the gas turns invisible as it "cools"
    for ch, damping, fade in (("smoke", 0.02, 0.0), ("velocity", 0.1, 0.5), ("temperature", 0.0, 0.0)):
        p = st.DefinePrim(f"/DemoFX/Gas/flowSimulate/advection/{ch}", "FlowAdvectionChannelParams")
        flow_attr(p, "damping", T.Float, damping)
        flow_attr(p, "fade", T.Float, fade)
    vort = st.DefinePrim("/DemoFX/Gas/flowSimulate/vorticity", "FlowVorticityParams")
    flow_attr(vort, "forceScale", T.Float, 0.6)
    alloc = st.DefinePrim("/DemoFX/Gas/flowSimulate/summaryAllocate", "FlowSummaryAllocateParams")
    flow_attr(alloc, "smokeThreshold", T.Float, 0.005)

    off = st.DefinePrim("/DemoFX/Gas/flowOffscreen", "FlowOffscreen")
    flow_attr(off, "layer", T.Int, 0)
    cmap = st.DefinePrim("/DemoFX/Gas/flowOffscreen/colormap", "FlowRayMarchColormapParams")
    r, g, b = GAS_COLOR
    # colormap x = smoke density: bright green already at low density (room haze ~0.2), alpha ramps quickly
    flow_attr(cmap, "xPoints", T.FloatArray, Vt.FloatArray([0.0, 0.04, 0.15, 0.4, 1.0]))
    flow_attr(cmap, "rgbaPoints", T.Float4Array, Vt.Vec4fArray([
        Gf.Vec4f(0, 0, 0, 0), Gf.Vec4f(r * 0.7, g * 0.7, b * 0.7, 0.4), Gf.Vec4f(r, g, b, 0.7),
        Gf.Vec4f(r, g, b, 0.9), Gf.Vec4f(r * 1.3, g * 1.3, b * 1.3, 1.0)]))
    flow_attr(cmap, "colorScale", T.Float, 1.5)
    shadow = st.DefinePrim("/DemoFX/Gas/flowOffscreen/shadow", "FlowShadowParams")
    flow_attr(shadow, "attenuation", T.Float, 1.0)
    flow_attr(shadow, "lightDirection", T.Float3, Gf.Vec3f(0.3, 0.3, 1.0))

    ren = st.DefinePrim("/DemoFX/Gas/flowRender", "FlowRender")
    flow_attr(ren, "layer", T.Int, 0)
    rm = st.DefinePrim("/DemoFX/Gas/flowRender/rayMarch", "FlowRayMarchParams")
    flow_attr(rm, "attenuation", T.Float, 1.0)
    flow_attr(rm, "stepSizeScale", T.Float, 0.75)

    def emitter(path, typ, on_at, smoke_rate, temp, vel):
        e = st.DefinePrim(path, typ)
        flow_attr(e, "layer", T.Int, 0)
        flow_attr(e, "smoke", T.Float, 1.0)
        flow_attr(e, "coupleRateSmoke", T.Float, smoke_rate)
        flow_attr(e, "temperature", T.Float, temp)
        flow_attr(e, "coupleRateTemperature", T.Float, 2.0)
        flow_attr(e, "fuel", T.Float, 0.0)
        flow_attr(e, "coupleRateFuel", T.Float, 0.0)
        flow_attr(e, "velocity", T.Float3, Gf.Vec3f(*vel))
        flow_attr(e, "velocityIsWorldSpace", T.Bool, True)
        flow_attr(e, "coupleRateVelocity", T.Float, 2.0)
        set_switch(e.CreateAttribute("enabled", T.Bool, custom=False), False, True, on_at)
        return e

    spill = emitter("/DemoFX/Gas/SpillFumes", "FlowEmitterSphere", T_SPILL_GAS, 6.0, 0.8, (0, 0, 0.4))
    flow_attr(spill, "position", T.Float3, Gf.Vec3f(*LANDING))
    flow_attr(spill, "radius", T.Float, 0.25)
    flow_attr(spill, "radiusIsWorldSpace", T.Bool, True)
    for name, (centre, half) in ROOM_VENTS.items():
        v = emitter(f"/DemoFX/Gas/{name}", "FlowEmitterBox", T_ROOM_GAS, 4.0, 0.7, (0, 0, 0.3))
        flow_attr(v, "position", T.Float3, Gf.Vec3f(*centre))
        flow_attr(v, "halfSize", T.Float3, Gf.Vec3f(*half))
    # room haze: smoke 0.15 x alpha(temp 0.5) ~0.9 x attenuation 1 = 0.14/m -> ~33 % transmission over 8 m
    fill = emitter("/DemoFX/Gas/RoomFill", "FlowEmitterBox", T_ROOM_GAS, 0.6, 0.5, (0, 0, 0.02))   # ~3 s to fill
    flow_attr(fill, "smoke", T.Float, 0.15)
    flow_attr(fill, "position", T.Float3, Gf.Vec3f(*ROOM_FILL[0]))
    flow_attr(fill, "halfSize", T.Float3, Gf.Vec3f(*ROOM_FILL[1]))


def make_siren(st):
    from pxr import UsdLux
    st.DefinePrim("/DemoFX/Siren", "Xform")
    period = round(FPS / BEACON_HZ)
    end = int(st.GetEndTimeCode())
    start = round(T_SIREN * FPS)
    for name, (pos, phase) in BEACONS.items():
        light = UsdLux.SphereLight.Define(st, f"/DemoFX/Siren/{name}")
        light.AddTranslateOp().Set(Gf.Vec3d(*pos))
        light.CreateRadiusAttr(0.15)
        light.CreateColorAttr(Gf.Vec3f(1.0, 0.05, 0.02))
        inten = light.CreateIntensityAttr()
        dome_mat = UsdShade.Material.Define(st, f"/Looks/{name}Dome")
        sh = UsdShade.Shader.Define(st, f"/Looks/{name}Dome/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.6, 0.02, 0.02))
        emis = sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f)
        dome_mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        dome = UsdGeom.Sphere.Define(st, f"/DemoFX/Siren/{name}/Dome")
        dome.CreateRadiusAttr(0.10)
        UsdShade.MaterialBindingAPI.Apply(dome.GetPrim()).Bind(dome_mat)
        # square wave (on half a period, off half), phase-shifted per strobe so the two alternate
        for attr, on, off in ((inten, BEACON_INTENSITY, 0.0), (emis, Gf.Vec3f(10, 0.3, 0.1), Gf.Vec3f(0))):
            attr.Set(off, Usd.TimeCode(0))
            attr.Set(off, Usd.TimeCode(start - 1))
            f = start + round(phase * period)
            if f > start:
                attr.Set(off, Usd.TimeCode(f - 1))
            on_frames = round(period * BEACON_DUTY)
            while f < end:
                attr.Set(on, Usd.TimeCode(f))
                attr.Set(on, Usd.TimeCode(f + on_frames - 1))
                attr.Set(off, Usd.TimeCode(f + on_frames))
                attr.Set(off, Usd.TimeCode(f + period - 1))
                f += period

    # existing lab lights: drop to emergency level and go red at T_SIREN
    for path, (factor, color) in LAB_LIGHTS.items():
        prim = st.GetPrimAtPath(path)
        assert prim.IsValid(), f"lab light missing: {path}"
        light = UsdLux.LightAPI(prim)
        i0 = light.GetIntensityAttr().Get()
        c0 = light.GetColorAttr().Get() or Gf.Vec3f(1)
        set_switch(light.GetIntensityAttr(), i0, i0 * factor, T_SIREN)
        set_switch(light.GetColorAttr(), c0, Gf.Vec3f(*color), T_SIREN)


# ---------------------------------------------------------------------------
def build():
    layer = Sdf.Layer.CreateNew(DEMO) if not os.path.exists(DEMO) else Sdf.Layer.FindOrOpen(DEMO)
    layer.Clear()
    layer.subLayerPaths = ["./chem_lab_scene.usda"]
    st = Usd.Stage.Open(layer)
    st.SetEditTarget(layer)
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    st.SetDefaultPrim(st.GetPrimAtPath("/World"))
    st.SetTimeCodesPerSecond(60)
    st.SetStartTimeCode(0)
    st.SetEndTimeCode(FPS * 600)   # 10 min: siren/gas keyframes run to the end, so the effects never reset mid-demo
    # start view = the saved /Cameras/LabView (customLayerData is not composed from sublayers, so restate it here)
    persp = {"position": Gf.Vec3d(*CAMERA_POS), "target": Gf.Vec3d(*CAMERA_TARGET)}
    layer.customLayerData = {
        "comment": "built by build_demo_layer.py (sublayer: chem_lab_scene.usda)",
        "cameraSettings": {"Perspective": persp, "boundCamera": "/OmniverseKit_Persp"},
        "renderSettings": {"rtx:flow:enabled": True, "rtx:flow:rayTracedTranslucencyEnabled": True,
                           "rtx:flow:rayTracedReflectionsEnabled": True, "rtx:flow:pathTracingEnabled": True},
    }

    # sanity: the bench module is still where the placements assume
    r = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(
        st.GetPrimAtPath("/ChemLab/ST_Furn_ScienceDesk01_mo_001")).ComputeAlignedRange()
    assert abs(r.GetMin()[0] - (-5.265)) < 0.01 and abs(r.GetMax()[2] - (-1.644)) < 0.01, f"bench moved: {r}"

    # particle fluid needs GPU dynamics (override on the scene's physicsScene, in this layer only)
    scene = PhysxSchema.PhysxSceneAPI.Apply(st.GetPrimAtPath(PHYSICS_SCENE))
    scene.CreateEnableGPUDynamicsAttr().Set(True)
    scene.CreateBroadphaseTypeAttr().Set("GPU")

    # the base scene's empty beaker lying in front of the robot is replaced by the hazard beaker
    st.GetPrimAtPath("/Beaker").SetActive(False)

    st.DefinePrim("/DemoBeakers", "Xform")
    z = DESK_TOP + 0.001
    for name, xy in DESK_BEAKERS.items():
        make_beaker(st, f"/DemoBeakers/{name}", xy, z + ON_BALANCE.get(name, 0.0), sdf_res=DESK_SDF_RES)
    make_props(st)

    hz = make_beaker(st, "/DemoBeakers/HazardBeaker", HAZARD_XY, z)
    UsdPhysics.RigidBodyAPI(hz).CreateVelocityAttr(Gf.Vec3f(*HAZARD_VEL))
    make_cup(st, "/DemoBeakers/HazardBeaker")
    n = make_fluid(st, HAZARD_XY, z, HAZARD_VEL)
    make_desk_liquids(st)
    make_gas(st)
    make_siren(st)

    layer.Save()
    print(f"wrote {DEMO}")
    print(f"  {len(DESK_BEAKERS)} desk beakers upright at z={z}")
    print(f"  hazard beaker at {HAZARD_XY}, v0={HAZARD_VEL}, {n} fluid particles")
    print(f"  timeline: fumes {T_SPILL_GAS}s, siren {T_SIREN}s, room vents {T_ROOM_GAS}s  (Flow needs: isaacsim ... --enable omni.flowusd)")


def test(seconds=4.0):
    """Headless: open the demo, play, report where the hazard beaker and the fluid end up."""
    import omni.timeline
    import omni.usd

    assert omni.usd.get_context().open_stage(DEMO)
    for _ in range(10):
        app.update()
    st = omni.usd.get_context().get_stage()
    xc = UsdGeom.XformCache()
    names = [c.GetName() for c in st.GetPrimAtPath("/DemoBeakers").GetChildren()]
    print("=== props:", [c.GetName() for c in st.GetPrimAtPath("/DemoProps").GetChildren()])

    def pos(n):
        xc.Clear()
        return xc.GetLocalToWorldTransform(st.GetPrimAtPath(f"/DemoBeakers/{n}")).ExtractTranslation()

    def fluid_stats():
        pts = st.GetPrimAtPath("/DemoFX/HazardFluid").GetAttribute("points").Get()
        c = pos("HazardBeaker")
        near = sum(1 for p in pts if (Gf.Vec3d(p) - c).GetLength() < 0.12)
        zs = sorted(p[2] for p in pts)
        return len(pts), near, zs[0], zs[len(zs) // 2], zs[-1]

    tl = omni.timeline.get_timeline_interface()
    tl.play()
    for t_report in (0.3, 0.8, 1.5, seconds):
        while tl.get_current_time() < t_report:
            app.update()
        print(f"=== t={tl.get_current_time():.2f}s")
        for n in names:
            p = pos(n)
            print(f"   {n:14s} ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
        n_all, near, zmin, zmed, zmax = fluid_stats()
        print(f"   fluid: {n_all} particles, {near} within 12 cm of beaker, z min/med/max {zmin:.3f}/{zmed:.3f}/{zmax:.3f}")
    tl.stop()


if __name__ == "__main__":
    build()
    if "--test" in sys.argv:
        test()
    app.close()
