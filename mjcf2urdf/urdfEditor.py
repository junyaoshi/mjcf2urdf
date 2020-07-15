from pybullet_utils.urdfEditor import UrdfEditor, UrdfVisual, UrdfCollision
import pybullet as p


class UrdfEditorNew(UrdfEditor):
    def saveUrdf(self, fileName, saveVisuals=True):
        file = open(fileName, "w")
        file.write("<?xml version=\"1.0\" ?>\n")
        file.write("<robot name=\"")
        file.write(self.robotName)
        file.write("\">\n")

        for link in self.urdfLinks:
            self.writeLink(file, link, saveVisuals)

        for joint in self.urdfJoints:
            self.writeJoint(file, joint)

        file.write("</robot>\n")
        file.close()

    def convertLinkFromMultiBody(self, bodyUid, linkIndex, urdfLink, physicsClientId):
        dyn = p.getDynamicsInfo(bodyUid, linkIndex, physicsClientId=physicsClientId)
        urdfLink.urdf_inertial.mass = dyn[0]
        urdfLink.urdf_inertial.inertia_xxyyzz = dyn[2]
        urdfLink.urdf_inertial.origin_xyz = dyn[3]
        rpy = p.getEulerFromQuaternion(dyn[4])
        urdfLink.urdf_inertial.origin_rpy = rpy

        visualShapes = p.getVisualShapeData(bodyUid, physicsClientId=physicsClientId)
        matIndex = 0
        for i in range(len(visualShapes)):
            v = visualShapes[i]
            if (v[1] == linkIndex and v[2] == p.GEOM_MESH):
                urdfVisual = UrdfVisual()
                urdfVisual.geom_type = v[2]
                urdfVisual.geom_meshfilename = v[4].decode("utf-8")
                urdfVisual.geom_meshscale = v[3]
                urdfVisual.origin_xyz = v[5]
                urdfVisual.origin_rpy = p.getEulerFromQuaternion(v[6])
                urdfVisual.material_rgba = v[7]
                name = 'mat_{}_{}'.format(linkIndex, matIndex)
                urdfVisual.material_name = name
                urdfLink.urdf_visual_shapes.append(urdfVisual)
                matIndex = matIndex + 1

        collisionShapes = p.getCollisionShapeData(bodyUid, linkIndex, physicsClientId=physicsClientId)
        for i in range(len(collisionShapes)):
            v = collisionShapes[i]
            if (v[2] == p.GEOM_MESH):
                urdfCollision = UrdfCollision()
                urdfCollision.geom_type = v[2]
                urdfCollision.geom_meshfilename = urdfLink.urdf_visual_shapes[i].geom_meshfilename
                urdfCollision.geom_meshscale = v[3]
                pos, orn = p.multiplyTransforms(dyn[3], dyn[4], \
                                                v[5], v[6])
                urdfCollision.origin_xyz = pos
                urdfCollision.origin_rpy = p.getEulerFromQuaternion(orn)
                urdfLink.urdf_collision_shapes.append(urdfCollision)

