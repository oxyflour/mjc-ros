import mujoco
import numpy as np
import mitsuba as mi
import os

XML_OUT = "scene_mitsuba.xml"
EXPORT_DIR = "mujoco_export"
os.makedirs(EXPORT_DIR, exist_ok=True)

model = mujoco.MjModel.from_xml_path("data/mujoco_menagerie-main/universal_robots_ur5e/ur5e.xml")
data = mujoco.MjData(model)

def quat_to_mat(quat):
    mat = np.zeros((9, ))
    mujoco.mju_quat2Mat(mat, quat)
    return mat.reshape((3, 3))

def export_mesh(model, geom_id):
    geom = model.geom(geom_id)
    if geom.type != mujoco.mjtGeom.mjGEOM_MESH:
        return None

    mesh_id = geom.dataid
    # vertex
    v_start, = model.mesh_vertadr[mesh_id]
    v_count, = model.mesh_vertnum[mesh_id]
    verts = [model.mesh_vert[i] for i in range(v_start, v_start + v_count)]

    # faces
    f_start, = model.mesh_faceadr[mesh_id]
    f_count, = model.mesh_facenum[mesh_id]
    faces = [model.mesh_face[i] for i in range(f_start, f_start + f_count)]

    filename = os.path.join(EXPORT_DIR, f"mesh_{geom_id}.obj")
    with open(filename, "w") as f:
        for v in verts:
            f.write(f"v {v[0]} {v[1]} {v[2]}\n")
        for tri in faces:
            f.write(f"f {tri[0]+1} {tri[1]+1} {tri[2]+1}\n")

    return filename


mitsuba_shapes = []

for i in range(model.ngeom):
    geom_type = model.geom_type[i]
    pos = model.geom_pos[i]
    quat = model.geom_quat[i]
    rgba = model.geom_rgba[i]

    # transform
    R = quat_to_mat(quat)
    T = np.eye(4)
    T[:3,:3] = R
    T[:3, 3] = pos

    transform_str = " ".join(map(str, T.flatten()))

    # mesh 或 primitive
    if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
        obj_path = export_mesh(model, i)
        shape = f"""
        <shape type="obj">
            <string name="filename" value="{obj_path}"/>
            <transform name="to_world">
                <matrix value="{transform_str}"/>
            </transform>
            <bsdf type="diffuse">
                <rgb name="reflectance" value="{rgba[0]} {rgba[1]} {rgba[2]}"/>
            </bsdf>
        </shape>
        """
    else:
        # sphere/box/capsule → 用 mitsuba primitive
        size = model.geom_size[i]
        radius = size[0]

        if geom_type == mujoco.mjtGeom.mjGEOM_SPHERE:
            shape = f"""
            <shape type="sphere">
                <float name="radius" value="{radius}"/>
                <transform name="to_world">
                    <matrix value="{transform_str}"/>
                </transform>
                <bsdf type="diffuse">
                    <rgb name="reflectance" value="{rgba[0]} {rgba[1]} {rgba[2]}"/>
                </bsdf>
            </shape>
            """

        else:
            continue # demo: 如需要 box/capsule，我可以继续写

    mitsuba_shapes.append(shape)


xml = f"""
<scene version="3.0.0">
    <integrator type="path"/>
    
    <emitter type="constant">
        <rgb name="radiance" value="1.0"/>
    </emitter>
    
    <sensor type="perspective">
        <transform name="to_world">
            <lookat origin="3 3 2" target="0 0 0" up="0 0 1"/>
        </transform>
        <float name="fov" value="45"/>
        <sampler type="independent">
            <integer name="sample_count" value="256"/>
        </sampler>
    </sensor>

    {''.join(mitsuba_shapes)}

</scene>
"""

with open(XML_OUT, "w") as f:
    f.write(xml)

print("Mitsuba scene exported:", XML_OUT)
