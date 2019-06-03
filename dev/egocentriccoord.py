# -*- coding: utf-8 -*-
"""
Created on Thu Feb 21 21:44:27 2019

@author: Pichau
"""

import numpy as np
import mathutils
import time
import skeletonmap

class EgocentricCoordinate:
    """
    Objects of this class holds the egocentric coordinates of a joint. It contains the joint, its name,
    and a list of reference (length = frame) for the coordinates data of that joint for every frame.
    """
    egolist = []

    def __init__(self, joint):
        self.joint = joint
        self.name = joint.name
        self.egolist.append(self)
        self.framecoord = []
        self.target = []

    def reset(self):
        """
        Clear all the coordinate data of every frame, but not this class instance
        """
        self.framecoord = []

    def addCoordFrame(self, frame):
        """
        Create a CoordFrame object to hold the egocentric coordinate data for a new frame
        """
        coord = CoordFrame(frame)
        self.framecoord.append(coord)
        return coord

    def getCoordFrame(self, framedesired):
        """
        Return the CoordFrame object that holds the data in the frame desired
        """
        if self.framecoord[framedesired].frame == framedesired:
            return self.framecoord[framedesired]
        for coord in self.framecoord:
            if coord.frame == framedesired:
                return coord

    def getTarget(self, frame):
        coord = self.getCoordFrame(frame)
        if coord:
            return coord.importance.dot(coord.targets)
        else:
            raise Exception('Egocentric Coordinates unavailable for this frame')

    @classmethod
    def getCoord(cls, jointname):
        for ego in cls.egolist:
            if jointname == ego.name:
                return ego
        print('Egocentric Coordinates not found')

    @classmethod
    def clean(cls):
        cls.egolist = []


class CoordFrame:

    def __init__(self, frame):
        self.frame = frame
        self.importance = [] #lambda
        self.refpoint = [] #x
        self.dispvector = [] #v
        self.normcoef = [] #C
        self.angle = [] #B
        self.distroot = [] #path distance to root
        self.triangle = [] #triangulo associado a essa coordenada
        self.normal = []
        self.targets = []

        self.tau = []#debbug tau
        self.ortho = [] #debbug importance
        self.proxi = [] #debbug importance


def importanceCalc(dispvector, normal):
        """
        Compute the importance factor for a given mesh component

        Parameters
        ----------
        dispvector : numpy array
                    Displacement vector. The vector from the component's reference point to the joint.

        normal : numpy array
                Component's normal.
        """
        epsilon = 0.01
        normdispvector = np.linalg.norm(dispvector)
        if normdispvector <= epsilon:
            proximity = 1/epsilon
        else:
            proximity = 1/normdispvector
        normal_unit = normal/np.linalg.norm(normal)
        dispvector_unit = dispvector/normdispvector
        orthogonality = np.clip(np.dot(normal_unit, dispvector_unit), -1.0, 1.0)
        if orthogonality < epsilon:
            orthogonality = epsilon
        orthogonality = np.abs(orthogonality)
        return orthogonality*proximity, orthogonality, proximity

def getVectors(animation, frame):
        """
        Get vectors to calculate the kinematic path

        :type animation: pyanimation.Animation
        :param animation: Animation (skeleton) to get the distance between mapped joints
        """
        skmap = animation.getskeletonmap()

        lvec_fore = skmap.vecLForearm(frame)
        rvec_fore = skmap.vecRForearm(frame)

        lvec_arm = skmap.vecLArm(frame)
        rvec_arm = skmap.vecRArm(frame)

        lvec_clavicle = skmap.vecLClavicle(frame)
        rvec_clavicle = skmap.vecRClavicle(frame)

        vec_neck = skmap.vecNeck(frame)

        vec_spine = skmap.vecSpine(frame)

        lvec_femur = skmap.vecLFemur(frame)
        rvec_femur = skmap.vecRFemur(frame)

        lvec_upleg = skmap.vecLUpleg(frame)
        rvec_upleg = skmap.vecRUpleg(frame)

        lvec_lowleg = skmap.vecLLowleg(frame)
        rvec_lowleg = skmap.vecRLowleg(frame)

        return lvec_fore, rvec_fore, lvec_arm, rvec_arm, lvec_clavicle, rvec_clavicle, vec_neck, vec_spine, lvec_femur, rvec_femur, lvec_upleg, rvec_upleg, lvec_lowleg, rvec_lowleg

#def getPosition(animation, frame):
#    """
#    Get positions in thecurrent frame for mapped joints
#
#    :type animation: pyanimation.Animation
#    :param animation: Animation (skeleton) to get the distance between mapped joints
#
#    :type frame: int
#    :param frame: Frame to get the positions
#    """
#    skmap = animation.getskeletonmap()
#    joints = skmap.getJointsLimbsHead()
#    pos = []
#    for joint in joints:
#        joint.getPosition(frame)


def DenormEgoLimb(joint, animation, surface, frame, vectors, egocoord, index):
        """
        Denormalize egocentric coordinates for the Limbs
        """
        assert joint is not None
        assert animation is not None
        assert surface is not None
        assert frame is not None
        assert vectors is not None
        assert index is not None
        lvec_fore, rvec_fore, lvec_arm, rvec_arm, lvec_clavicle, rvec_clavicle, vec_neck, vec_spine, lvec_femur, rvec_femur, lvec_upleg, rvec_upleg, lvec_lowleg, rvec_lowleg = vectors
        if joint == animation.getskeletonmap().rhand:
            #Right hand in respect to
            #LEFT FOREARM LIMB
            p0 = animation.getskeletonmap().lforearm.getPosition(frame)
            p1 = animation.getskeletonmap().lhand.getPosition(frame)
            r = surface.getPoint('foreLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [lvec_arm, lvec_clavicle, rvec_clavicle, rvec_arm, rvec_fore]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT ARM LIMB
            index += 1
            p0 = animation.getskeletonmap().larm.getPosition(frame)
            p1 = animation.getskeletonmap().lforearm.getPosition(frame)
            r = surface.getPoint('armLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [lvec_clavicle, rvec_clavicle, rvec_arm, rvec_fore]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT LOW LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().rlowleg.getPosition(frame)
            p1 = animation.getskeletonmap().rfoot.getPosition(frame)
            r = surface.getPoint('shinRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [rvec_upleg, rvec_femur, vec_spine, rvec_clavicle, rvec_arm, rvec_fore]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT UP LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().rupleg.getPosition(frame)
            p1 = animation.getskeletonmap().rlowleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [rvec_femur, vec_spine, rvec_clavicle, rvec_arm, rvec_fore]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT LOW LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [lvec_upleg, lvec_femur, vec_spine, rvec_clavicle, rvec_arm, rvec_fore]
            tau = 0
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT UP LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            p1 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('thightLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [lvec_femur, vec_spine, rvec_clavicle, rvec_arm, rvec_fore]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal

        elif joint == animation.getskeletonmap().lhand:
            #Left hand in respect to
            #RIGHT FOREARM LIMB
            p0 = animation.getskeletonmap().rforearm.getPosition(frame)
            p1 = animation.getskeletonmap().rhand.getPosition(frame)
            r = surface.getPoint('foreRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [rvec_arm, rvec_clavicle, lvec_clavicle, lvec_arm, lvec_fore]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT ARM LIMB
            index += 1
            p0 = animation.getskeletonmap().rarm.getPosition(frame)
            p1 = animation.getskeletonmap().rforearm.getPosition(frame)
            r = surface.getPoint('armRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [rvec_clavicle, lvec_clavicle, lvec_arm, lvec_fore]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT LOW LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().rlowleg.getPosition(frame)
            p1 = animation.getskeletonmap().rfoot.getPosition(frame)
            r = surface.getPoint('shinRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [rvec_upleg, rvec_femur, vec_spine, lvec_clavicle, lvec_arm, lvec_fore]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT UP LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().rupleg.getPosition(frame)
            p1 = animation.getskeletonmap().rlowleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [rvec_femur, vec_spine, lvec_clavicle, lvec_arm, lvec_fore]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT LOW LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [lvec_upleg, lvec_femur, vec_spine, lvec_clavicle, lvec_arm, lvec_fore]
            tau = 0
            for coef,vector in zip(egocoord.normcoef[index],path):
                tau += np.linalg.norm(vector)*coef
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT UP LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            p1 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('thightLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [lvec_femur, vec_spine, lvec_clavicle, lvec_arm, lvec_fore]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal

        elif joint == animation.getskeletonmap().rforearm:
            #Right elbow in respect to
            #LEFT FOREARM LIMB
            p0 = animation.getskeletonmap().lforearm.getPosition(frame)
            p1 = animation.getskeletonmap().lhand.getPosition(frame)
            r = surface.getPoint('foreLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [lvec_arm, lvec_clavicle, rvec_clavicle, rvec_arm]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT ARM LIMB
            index += 1
            p0 = animation.getskeletonmap().larm.getPosition(frame)
            p1 = animation.getskeletonmap().lforearm.getPosition(frame)
            r = surface.getPoint('armLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [lvec_clavicle, rvec_clavicle, rvec_arm]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT LOW LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().rlowleg.getPosition(frame)
            p1 = animation.getskeletonmap().rfoot.getPosition(frame)
            r = surface.getPoint('shinRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [rvec_upleg, rvec_femur, vec_spine, rvec_clavicle, rvec_arm]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT UP LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().rupleg.getPosition(frame)
            p1 = animation.getskeletonmap().rlowleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [rvec_femur, vec_spine, rvec_clavicle, rvec_arm]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT LOW LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [lvec_upleg, lvec_femur, vec_spine, rvec_clavicle, rvec_arm]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT UP LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            p1 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('thightLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [lvec_femur, vec_spine, rvec_clavicle, rvec_arm]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal

        elif joint == animation.getskeletonmap().lforearm:
            #Left elbow in respect to
            #RIGHT FOREARM LIMB
            p0 = animation.getskeletonmap().rforearm.getPosition(frame)
            p1 = animation.getskeletonmap().rhand.getPosition(frame)
            r = surface.getPoint('foreRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [rvec_arm, rvec_clavicle, lvec_clavicle, lvec_arm]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT ARM LIMB
            index += 1
            p0 = animation.getskeletonmap().rarm.getPosition(frame)
            p1 = animation.getskeletonmap().rforearm.getPosition(frame)
            r = surface.getPoint('armRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [rvec_clavicle, lvec_clavicle, lvec_arm]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT LOW LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().rlowleg.getPosition(frame)
            p1 = animation.getskeletonmap().rfoot.getPosition(frame)
            r = surface.getPoint('shinRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [rvec_upleg, rvec_femur, vec_spine, lvec_clavicle, lvec_arm]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT UP LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().rupleg.getPosition(frame)
            p1 = animation.getskeletonmap().rlowleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [rvec_femur, vec_spine, lvec_clavicle, lvec_arm]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT LOW LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [lvec_upleg, lvec_femur, vec_spine, lvec_clavicle, lvec_arm]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT UP LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            p1 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('thightLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = [lvec_femur, vec_spine, lvec_clavicle, lvec_arm]
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal

        elif joint == animation.getskeletonmap().rfoot:
            #Right foot in respect to
            #RIGHT FOREARM LIMB
            p0 = animation.getskeletonmap().rforearm.getPosition(frame)
            p1 = animation.getskeletonmap().rhand.getPosition(frame)
            r = surface.getPoint('foreRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- rvec_arm, - rvec_clavicle, - vec_spine, rvec_femur, rvec_upleg, rvec_lowleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT ARM LIMB
            index += 1
            p0 = animation.getskeletonmap().rarm.getPosition(frame)
            p1 = animation.getskeletonmap().rforearm.getPosition(frame)
            r = surface.getPoint('armRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- rvec_clavicle, - vec_spine, rvec_femur, rvec_upleg, rvec_lowleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT FOREARM LIMB
            index += 1
            p0 = animation.getskeletonmap().lforearm.getPosition(frame)
            p1 = animation.getskeletonmap().lhand.getPosition(frame)
            r = surface.getPoint('foreLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- lvec_arm, - lvec_clavicle, - vec_spine, rvec_femur, rvec_upleg, rvec_lowleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT ARM LIMB
            index += 1
            p0 = animation.getskeletonmap().larm.getPosition(frame)
            p1 = animation.getskeletonmap().lforearm.getPosition(frame)
            r = surface.getPoint('armLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- lvec_clavicle, - vec_spine, rvec_femur, rvec_upleg, rvec_lowleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT LOW LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([ - lvec_upleg,- lvec_femur, rvec_femur, rvec_upleg, rvec_lowleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT UP LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            p1 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- lvec_femur, rvec_femur, rvec_lowleg, rvec_upleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
        elif joint == animation.getskeletonmap().lfoot:
            #Left foot in respect to
            #RIGHT FOREARM LIMB
            p0 = animation.getskeletonmap().rforearm.getPosition(frame)
            p1 = animation.getskeletonmap().rhand.getPosition(frame)
            r = surface.getPoint('foreRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- rvec_arm, - rvec_clavicle, - vec_spine, lvec_femur, lvec_upleg, lvec_lowleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT ARM LIMB
            index += 1
            p0 = animation.getskeletonmap().rarm.getPosition(frame)
            p1 = animation.getskeletonmap().rforearm.getPosition(frame)
            r = surface.getPoint('armRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- rvec_clavicle, - vec_spine, lvec_femur, lvec_upleg, lvec_lowleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT FOREARM LIMB
            index += 1
            p0 = animation.getskeletonmap().lforearm.getPosition(frame)
            p1 = animation.getskeletonmap().lhand.getPosition(frame)
            r = surface.getPoint('foreLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- lvec_arm, - lvec_clavicle, - vec_spine, lvec_femur, lvec_upleg, lvec_lowleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT ARM LIMB
            index += 1
            p0 = animation.getskeletonmap().larm.getPosition(frame)
            p1 = animation.getskeletonmap().lforearm.getPosition(frame)
            r = surface.getPoint('armLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- lvec_clavicle, - vec_spine, lvec_femur, lvec_upleg, lvec_lowleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT LOW LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([ - lvec_upleg,- lvec_femur, lvec_femur, lvec_upleg, lvec_lowleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT UP LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            p1 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- lvec_femur, lvec_femur, lvec_upleg, lvec_lowleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
        elif joint == animation.getskeletonmap().rlowleg:
            #Right knee in respect to
            #RIGHT FOREARM LIMB
            p0 = animation.getskeletonmap().rforearm.getPosition(frame)
            p1 = animation.getskeletonmap().rhand.getPosition(frame)
            r = surface.getPoint('foreRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- rvec_arm, - rvec_clavicle, - vec_spine, rvec_femur, rvec_upleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT ARM LIMB
            index += 1
            p0 = animation.getskeletonmap().rarm.getPosition(frame)
            p1 = animation.getskeletonmap().rforearm.getPosition(frame)
            r = surface.getPoint('armRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- rvec_clavicle, - vec_spine, rvec_femur, rvec_upleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT FOREARM LIMB
            index += 1
            p0 = animation.getskeletonmap().lforearm.getPosition(frame)
            p1 = animation.getskeletonmap().lhand.getPosition(frame)
            r = surface.getPoint('foreLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- lvec_arm, - lvec_clavicle, - vec_spine, rvec_femur, rvec_upleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT ARM LIMB
            index += 1
            p0 = animation.getskeletonmap().larm.getPosition(frame)
            p1 = animation.getskeletonmap().lforearm.getPosition(frame)
            r = surface.getPoint('armLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- lvec_clavicle, - vec_spine, rvec_femur, rvec_upleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT LOW LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([ - lvec_upleg,- lvec_femur, rvec_femur, rvec_upleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT UP LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            p1 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- lvec_femur, rvec_femur, rvec_lowleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
        elif joint == animation.getskeletonmap().llowleg:
            #Left foot in respect to
            #RIGHT FOREARM LIMB
            p0 = animation.getskeletonmap().rforearm.getPosition(frame)
            p1 = animation.getskeletonmap().rhand.getPosition(frame)
            r = surface.getPoint('foreRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- rvec_arm, - rvec_clavicle, - vec_spine, lvec_femur, lvec_upleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #RIGHT ARM LIMB
            index += 1
            p0 = animation.getskeletonmap().rarm.getPosition(frame)
            p1 = animation.getskeletonmap().rforearm.getPosition(frame)
            r = surface.getPoint('armRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- rvec_clavicle, - vec_spine, lvec_femur, lvec_upleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT FOREARM LIMB
            index += 1
            p0 = animation.getskeletonmap().lforearm.getPosition(frame)
            p1 = animation.getskeletonmap().lhand.getPosition(frame)
            r = surface.getPoint('foreLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- lvec_arm, - lvec_clavicle, - vec_spine, lvec_femur, lvec_upleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT ARM LIMB
            index += 1
            p0 = animation.getskeletonmap().larm.getPosition(frame)
            p1 = animation.getskeletonmap().lforearm.getPosition(frame)
            r = surface.getPoint('armLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- lvec_clavicle, - vec_spine, lvec_femur, lvec_upleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT LOW LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([ - lvec_upleg,- lvec_femur, lvec_femur, lvec_upleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal
            #LEFT UP LEG LIMB
            index += 1
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            p1 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            de_refpoint, normal = mathutils.capsuleCartesian(egocoord.refpoint[index], p0, p1, r)
            path = np.asarray([- lvec_femur, lvec_femur, lvec_upleg])
            tau = (np.linalg.norm(path, axis=1)*egocoord.normcoef[index]).sum()
            de_displacement= egocoord.dispvector[index]*tau
            yield de_displacement, de_refpoint, tau, normal

def extremityNormal(animation, joint, frame):
        """
        Returns the surface normal

        Estimate the direction of a surface normal for the extrimity joints (hands and feet).
        Based on the TPose in frame = 0, the initial surface normal is computed through:
        Get the direction of the bone in the first frame (not the joint's orientation!)
        Set a rotation axis equal to the cross product of this direction and the Y-axis [0,1,0]
        The initial surface normal is the result of a 90 degrees rotation around this axis.

        With the initial surface normal computed, apply the same transforms of the joint
        in the initial surface normal, resulting in the current surface normal.
        """
        skmap = animation.getskeletonmap()
        try:
            initnormal = joint.initNormal
        except:
            #The joint still does not have a initial normal
            #Get the direction of the bone
            if joint == skmap.rhand:
                child = skmap.rhandmiddle
                if not child:
                    print('Right hand middle base not mapped, using bone direction = [-1,0,0]')
                    bonedirection = [-1,0,0]
            elif joint == skmap.lhand:
                child = skmap.lhandmiddle
                if not child:
                    print('Left hand middle base not mapped, using bone direction = [1,0,0]')
                    bonedirection = [1,0,0]
            elif joint == skmap.rfoot:
                child = skmap.rtoebase
                if not child:
                    print('Right toe base not mapped, using bone direction = [0,0,1]')
                    bonedirection = [0,0,1]
            elif joint == skmap.lfoot:
                child = skmap.ltoebase
                if not child:
                    print('Left toe base not mapped, using bone direction = [0,0,1]')
                    bonedirection = [0,0,1]
            else:
                raise Exception('This is not a extrimity joint.')
            if child:
                bonedirection = child.getPosition(frame=0) - joint.getPosition(frame=0)
                bonedirection = mathutils.unitVector(bonedirection)
            #Get the rotation axis
            axis = np.cross( [0,1,0], bonedirection )
            #Get rotation matrix
            matrix = mathutils.matrixRotation(90, axis[0], axis[1], axis[2], shape = 3)
            initnormal = np.dot( matrix, bonedirection )
            initnormal = mathutils.unitVector(initnormal)
            joint.initNormal = initnormal[:]
        if frame == 0:
            return initnormal
        else:
            #Get the rotation from frame zero from current frame of the joint
            glbTransformMat = joint.getGlobalTransform(frame)
            glbRotationMat = mathutils.shape4ToShape3(glbTransformMat)
            glbInitTransformMat = joint.getGlobalTransform(frame = 0)
            glbInitRotationMat = mathutils.shape4ToShape3(glbInitTransformMat)
            transform = np.dot(glbRotationMat, glbInitRotationMat.T)
            #Rotate initial surface normal
            currentnormal = np.dot( transform, initnormal )

            return currentnormal

def importanceCalcLimb(vectors, limbname, dispvector, normal):
        """
        Compute the importance for the limbs (without the surface normal vector)
        """
        lvec_fore, rvec_fore, lvec_arm, rvec_arm, lvec_clavicle, rvec_clavicle, vec_neck, vec_spine, lvec_femur, rvec_femur, lvec_upleg, rvec_upleg, lvec_lowleg, rvec_lowleg = vectors
        if limbname == 'rarm':
            bone = rvec_arm
        elif limbname == 'larm':
            bone = lvec_arm
        elif limbname == 'rfore':
            bone = rvec_fore
        elif limbname == 'lfore':
            bone = lvec_fore
        elif limbname == 'rlowleg':
            bone = rvec_lowleg
        elif limbname == 'llowleg':
            bone = lvec_lowleg
        elif limbname == 'rupleg':
            bone = rvec_upleg
        elif limbname == 'lupleg':
            bone = lvec_upleg
        else:
            print('Unknown limb name')
            return None
#        dispvector_unit = dispvector/np.linalg.norm(dispvector)
        bone = bone/np.linalg.norm(bone)
        importance, orthogonality, proximity = importanceCalc(dispvector, normal)
        return importance, orthogonality, proximity


def pathnormCalcLimb(joint, animation, mesh, frame, refpoint, vectors, surface = None):
        lvec_fore, rvec_fore, lvec_arm, rvec_arm, lvec_clavicle, rvec_clavicle, vec_neck, vec_spine, lvec_femur, rvec_femur, lvec_upleg, rvec_upleg, lvec_lowleg, rvec_lowleg = vectors
#            TODO: Fazer para cada junta para cada um dos membros
        jointPosition = joint.getPosition(frame)
        if joint == animation.getskeletonmap().rhand:
            #Right hand in respect to
            #LEFT FOREARM LIMB
            p1 = animation.getskeletonmap().lhand.getPosition(frame)
            p0 = animation.getskeletonmap().lforearm.getPosition(frame)
            r = surface.getPoint('foreLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_arm, - lvec_clavicle, rvec_clavicle, rvec_arm, rvec_fore])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lfore', normal, cylindric, refpoint
            #LEFT ARM LIMB
            p1 = p0[:]
            p0 = animation.getskeletonmap().larm.getPosition(frame)
            r = surface.getPoint('armLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([ - lvec_clavicle, rvec_clavicle, rvec_arm, rvec_fore])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'larm', normal, cylindric, refpoint
            #RIGHT LOW LEG LIMB
            p1 = animation.getskeletonmap().rfoot.getPosition(frame)
            p0 = animation.getskeletonmap().rlowleg.getPosition(frame)
            r = surface.getPoint('shinRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_upleg, - rvec_femur, vec_spine, rvec_clavicle, rvec_arm, rvec_fore])
            vec_displacement = -(refpoint - p0)  + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rlowleg', normal, cylindric, refpoint
            #RIGHT UP LEG LIMB
            p1 = p0[:]
            p0 = animation.getskeletonmap().rupleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_femur, vec_spine, rvec_clavicle, rvec_arm, rvec_fore])
            vec_displacement = -(refpoint - p0)  + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rupleg', normal, cylindric, refpoint
            #LEFT LOW LEG LIMB
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_upleg, - lvec_femur, vec_spine, rvec_clavicle, rvec_arm, rvec_fore])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement,cos, tau, 'llowleg', normal, cylindric, refpoint
            #LEFT UP LEG LIMB
            p1 = p0[:]
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            r = surface.getPoint('thightLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_femur, vec_spine, rvec_clavicle, rvec_arm, rvec_fore])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lupleg', normal, cylindric, refpoint

        elif joint == animation.getskeletonmap().lhand:
            #Left hand in respect to
            #RIGHT FOREARM LIMB
            p1 = animation.getskeletonmap().rhand.getPosition(frame)
            p0 = animation.getskeletonmap().rforearm.getPosition(frame)
            r = surface.getPoint('foreRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_arm, - rvec_clavicle, lvec_clavicle, lvec_arm, lvec_fore])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rfore', normal, cylindric, refpoint
            #RIGHT ARM LIMB
            p1 = p0[:]
            p0 = animation.getskeletonmap().rarm.getPosition(frame)
            r = surface.getPoint('armRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_clavicle, lvec_clavicle, lvec_arm, lvec_fore])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rarm', normal, cylindric, refpoint
            #RIGHT LOW LEG LIMB
            p1 = animation.getskeletonmap().rfoot.getPosition(frame)
            p0 = animation.getskeletonmap().rlowleg.getPosition(frame)
            r = surface.getPoint('shinRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_upleg, - rvec_femur, vec_spine, lvec_clavicle, lvec_arm, lvec_fore])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rlowleg', normal, cylindric, refpoint
            #RIGHT UP LEG LIMB
            p1 = p0[:]
            p0 = animation.getskeletonmap().rupleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_femur, vec_spine, lvec_clavicle, lvec_arm, lvec_fore])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rupleg', normal, cylindric, refpoint
            #LEFT LOW LEG LIMB
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([ - lvec_upleg, - lvec_femur, vec_spine, lvec_clavicle, lvec_arm, lvec_fore])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'llowleg', normal, cylindric, refpoint
            #LEFT UP LEG LIMB
            p1 = p0[:]
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            r = surface.getPoint('thightLeft').radius
            path = np.asarray([- lvec_femur, vec_spine, lvec_clavicle, lvec_arm, lvec_fore])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lupleg', normal, cylindric, refpoint
        elif joint == animation.getskeletonmap().rforearm:
            #Right elbow in respect to
            #LEFT FOREARM LIMB
            p0 = animation.getskeletonmap().lforearm.getPosition(frame)
            p1 = animation.getskeletonmap().lhand.getPosition(frame)
            r = surface.getPoint('foreLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_arm, - lvec_clavicle, rvec_clavicle, rvec_arm])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lfore', normal, cylindric, refpoint
            #LEFT ARM LIMB
            p0 = animation.getskeletonmap().larm.getPosition(frame)
            p1 = animation.getskeletonmap().lforearm.getPosition(frame)
            r = surface.getPoint('armLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_clavicle, rvec_clavicle, rvec_arm])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'larm', normal, cylindric, refpoint
            #RIGHT LOW LEG LIMB
            p0 = animation.getskeletonmap().rlowleg.getPosition(frame)
            p1 = animation.getskeletonmap().rfoot.getPosition(frame)
            r = surface.getPoint('shinRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_upleg, - rvec_femur , vec_spine , rvec_clavicle , rvec_arm])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rlowleg', normal, cylindric, refpoint
            #RIGHT UP LEG LIMB
            p0 = animation.getskeletonmap().rupleg.getPosition(frame)
            p1 = animation.getskeletonmap().rlowleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_femur, vec_spine, rvec_clavicle, rvec_arm])
            vec_displacement = -(refpoint -p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rupleg', normal, cylindric, refpoint
            #LEFT LOW LEG LIMB
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_upleg, - lvec_femur, vec_spine, rvec_clavicle, rvec_arm])
            vec_displacement = -(refpoint - p0)+ path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'llowleg', normal, cylindric, refpoint
            #LEFT UP LEG LIMB
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            p1 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('thightLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_femur, vec_spine, rvec_clavicle, rvec_arm])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lupleg', normal, cylindric, refpoint
        elif joint == animation.getskeletonmap().lforearm:
            #Left elbow in respect to
            #RIGHT FOREARM LIMB
            p0 = animation.getskeletonmap().rforearm.getPosition(frame)
            p1 = animation.getskeletonmap().rhand.getPosition(frame)
            r = surface.getPoint('foreRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_arm, - rvec_clavicle, lvec_clavicle, lvec_arm])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement,cos, tau, 'rfore', normal, cylindric, refpoint
            #RIGHT ARM LIMB
            p0 = animation.getskeletonmap().rarm.getPosition(frame)
            p1 = animation.getskeletonmap().rforearm.getPosition(frame)
            r = surface.getPoint('armRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_clavicle, lvec_clavicle, lvec_arm])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rarm', normal, cylindric, refpoint
            #RIGHT LOW LEG LIMB
            p0 = animation.getskeletonmap().rlowleg.getPosition(frame)
            p1 = animation.getskeletonmap().rfoot.getPosition(frame)
            r = surface.getPoint('shinRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_upleg, - rvec_femur, vec_spine, lvec_clavicle, lvec_arm])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rlowleg', normal, cylindric, refpoint
            #RIGHT UP LEG LIMB
            p0 = animation.getskeletonmap().rupleg.getPosition(frame)
            p1 = animation.getskeletonmap().rlowleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_femur, vec_spine, lvec_clavicle, lvec_arm])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rupleg', normal, cylindric, refpoint
            #LEFT LOW LEG LIMB
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([ - lvec_upleg,- lvec_femur, vec_spine, lvec_clavicle, lvec_arm])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'llowleg', normal, cylindric, refpoint
            #LEFT UP LEG LIMB
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            p1 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('thightLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_femur, vec_spine, lvec_clavicle, lvec_arm])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lupleg', normal, cylindric, refpoint
        elif joint == animation.getskeletonmap().rfoot:
            #Right foot in respect to
            #RIGHT FOREARM LIMB
            p0 = animation.getskeletonmap().rforearm.getPosition(frame)
            p1 = animation.getskeletonmap().rhand.getPosition(frame)
            r = surface.getPoint('foreRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_arm, - rvec_clavicle, - vec_spine, rvec_femur, rvec_upleg, rvec_lowleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement,cos, tau, 'rfore', normal, cylindric, refpoint
            #RIGHT ARM LIMB
            p0 = animation.getskeletonmap().rarm.getPosition(frame)
            p1 = animation.getskeletonmap().rforearm.getPosition(frame)
            r = surface.getPoint('armRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_clavicle, - vec_spine, rvec_femur, rvec_upleg, rvec_lowleg])
            vec_displacement = -(refpoint - animation.getskeletonmap().rarm.getPosition(frame)) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rfore', normal, cylindric, refpoint
            #LEFT FOREARM LIMB
            p0 = animation.getskeletonmap().lforearm.getPosition(frame)
            p1 = animation.getskeletonmap().lhand.getPosition(frame)
            r = surface.getPoint('foreLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_arm, - lvec_clavicle, - vec_spine, rvec_femur, rvec_upleg, rvec_lowleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lfore', normal, cylindric, refpoint
            #LEFT ARM LIMB
            p0 = animation.getskeletonmap().larm.getPosition(frame)
            p1 = animation.getskeletonmap().lforearm.getPosition(frame)
            r = surface.getPoint('armLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_clavicle, - vec_spine, rvec_femur, rvec_upleg, rvec_lowleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'larm', normal, cylindric, refpoint
            #LEFT LOW LEG LIMB
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([ - lvec_upleg,- lvec_femur, rvec_femur, rvec_upleg, rvec_lowleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'llowleg', normal, cylindric, refpoint
            #LEFT UP LEG LIMB
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            p1 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('thightLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_femur, rvec_femur, rvec_lowleg, rvec_upleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lupleg', normal, cylindric, refpoint
        elif joint == animation.getskeletonmap().lfoot:
            #Left foot in respect to
            #RIGHT FOREARM LIMB
            p0 = animation.getskeletonmap().rforearm.getPosition(frame)
            p1 = animation.getskeletonmap().rhand.getPosition(frame)
            r = surface.getPoint('foreRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_arm, - rvec_clavicle, - vec_spine, lvec_femur, lvec_upleg, lvec_lowleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement,cos, tau, 'rfore', normal, cylindric, refpoint
            #RIGHT ARM LIMB
            p0 = animation.getskeletonmap().rarm.getPosition(frame)
            p1 = animation.getskeletonmap().rforearm.getPosition(frame)
            r = surface.getPoint('armRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_clavicle, - vec_spine, lvec_femur, lvec_upleg, lvec_lowleg])
            vec_displacement = -(refpoint - animation.getskeletonmap().rarm.getPosition(frame)) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rfore', normal, cylindric, refpoint
            #LEFT FOREARM LIMB
            p0 = animation.getskeletonmap().lforearm.getPosition(frame)
            p1 = animation.getskeletonmap().lhand.getPosition(frame)
            r = surface.getPoint('foreLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_arm, - lvec_clavicle, - vec_spine, lvec_femur, lvec_upleg, lvec_lowleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lfore', normal, cylindric, refpoint
            #LEFT ARM LIMB
            p0 = animation.getskeletonmap().larm.getPosition(frame)
            p1 = animation.getskeletonmap().lforearm.getPosition(frame)
            r = surface.getPoint('armLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_clavicle, - vec_spine, lvec_femur, lvec_upleg, lvec_lowleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'larm', normal, cylindric, refpoint
            #RIGHT LOW LEG LIMB
            p0 = animation.getskeletonmap().rlowleg.getPosition(frame)
            p1 = animation.getskeletonmap().rfoot.getPosition(frame)
            r = surface.getPoint('shinRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([ - rvec_upleg,- rvec_femur, lvec_femur, lvec_upleg, lvec_lowleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'llowleg', normal, cylindric, refpoint
            #RIGHT UP LEG LIMB
            p0 = animation.getskeletonmap().rupleg.getPosition(frame)
            p1 = animation.getskeletonmap().rlowleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_femur, lvec_femur, lvec_upleg, lvec_lowleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lupleg', normal, cylindric, refpoint
        elif joint == animation.getskeletonmap().rlowleg:
            #Right knee in respect to
            #RIGHT FOREARM LIMB
            p0 = animation.getskeletonmap().rforearm.getPosition(frame)
            p1 = animation.getskeletonmap().rhand.getPosition(frame)
            r = surface.getPoint('foreRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_arm, - rvec_clavicle, - vec_spine, rvec_femur, rvec_upleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement,cos, tau, 'rfore', normal, cylindric, refpoint
            #RIGHT ARM LIMB
            p0 = animation.getskeletonmap().rarm.getPosition(frame)
            p1 = animation.getskeletonmap().rforearm.getPosition(frame)
            r = surface.getPoint('armRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_clavicle, - vec_spine, rvec_femur, rvec_upleg])
            vec_displacement = -(refpoint - animation.getskeletonmap().rarm.getPosition(frame)) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rfore', normal, cylindric, refpoint
            #LEFT FOREARM LIMB
            p0 = animation.getskeletonmap().lforearm.getPosition(frame)
            p1 = animation.getskeletonmap().lhand.getPosition(frame)
            r = surface.getPoint('foreLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_arm, - lvec_clavicle, - vec_spine, rvec_femur, rvec_upleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lfore', normal, cylindric, refpoint
            #LEFT ARM LIMB
            p0 = animation.getskeletonmap().larm.getPosition(frame)
            p1 = animation.getskeletonmap().lforearm.getPosition(frame)
            r = surface.getPoint('armLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_clavicle, - vec_spine, rvec_femur, rvec_upleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'larm', normal, cylindric, refpoint
            #LEFT LOW LEG LIMB
            p0 = animation.getskeletonmap().llowleg.getPosition(frame)
            p1 = animation.getskeletonmap().lfoot.getPosition(frame)
            r = surface.getPoint('shinLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([ - lvec_upleg,- lvec_femur, rvec_femur, rvec_upleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'llowleg', normal, cylindric, refpoint
            #LEFT UP LEG LIMB
            p0 = animation.getskeletonmap().lupleg.getPosition(frame)
            p1 = animation.getskeletonmap().llowleg.getPosition(frame)
            r = surface.getPoint('thightLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_femur, rvec_femur, rvec_upleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lupleg', normal, cylindric, refpoint
        elif joint == animation.getskeletonmap().llowleg:
            #Left knee in respect to
            #RIGHT FOREARM LIMB
            p0 = animation.getskeletonmap().rforearm.getPosition(frame)
            p1 = animation.getskeletonmap().rhand.getPosition(frame)
            r = surface.getPoint('foreRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_arm, - rvec_clavicle, - vec_spine, lvec_femur, lvec_upleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement,cos, tau, 'rfore', normal, cylindric, refpoint
            #RIGHT ARM LIMB
            p0 = animation.getskeletonmap().rarm.getPosition(frame)
            p1 = animation.getskeletonmap().rforearm.getPosition(frame)
            r = surface.getPoint('armRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_clavicle, - vec_spine, lvec_femur, lvec_upleg])
            vec_displacement = -(refpoint - animation.getskeletonmap().rarm.getPosition(frame)) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'rfore', normal, cylindric, refpoint
            #LEFT FOREARM LIMB
            p0 = animation.getskeletonmap().lforearm.getPosition(frame)
            p1 = animation.getskeletonmap().lhand.getPosition(frame)
            r = surface.getPoint('foreLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_arm, - lvec_clavicle, - vec_spine, lvec_femur, lvec_upleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lfore', normal, cylindric, refpoint
            #LEFT ARM LIMB
            p0 = animation.getskeletonmap().larm.getPosition(frame)
            p1 = animation.getskeletonmap().lforearm.getPosition(frame)
            r = surface.getPoint('armLeft').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- lvec_clavicle, - vec_spine, lvec_femur, lvec_upleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'larm', normal, cylindric, refpoint
            #LEFT LOW LEG LIMB
            p0 = animation.getskeletonmap().rlowleg.getPosition(frame)
            p1 = animation.getskeletonmap().rfoot.getPosition(frame)
            r = surface.getPoint('shinRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([ - rvec_upleg,- rvec_femur, lvec_femur, lvec_upleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'llowleg', normal, cylindric, refpoint
            #LEFT UP LEG LIMB
            p0 = animation.getskeletonmap().rupleg.getPosition(frame)
            p1 = animation.getskeletonmap().rlowleg.getPosition(frame)
            r = surface.getPoint('thightRight').radius
            cylindric, refpoint, normal = mathutils.capsuleCollision(jointPosition,p0,p1,r)
            path = np.asarray([- rvec_femur, lvec_femur, lvec_upleg])
            vec_displacement = -(refpoint - p0) + path.sum(axis=0)
            cos = np.asarray([mathutils.cosBetween(vec_displacement, path[i]) for i in range(len(path))])
            tau = (np.linalg.norm(path, axis = 1)*cos).sum()
            yield vec_displacement, cos, tau, 'lupleg', normal, cylindric, refpoint


def AdjustExtremityOrientation(animation, surface, ego, sourceanim):
#    TODO: NOT WORKING
    #O calculo da superficie parece estar OK, então acredito que o erro esteja aqui
    lhand, rhand = animation.getskeletonmap().lhand, animation.getskeletonmap().rhand
    lfoot, rfoot = animation.getskeletonmap().lfoot, animation.getskeletonmap().rfoot
    headmesh = surface.headmesh
    bodymesh = surface.bodymesh

    start=time.time()
    print('Adjusting extremities orientation')
    for frame in range(animation.frames):
        vectors = getVectors(animation, frame)
        lvec_fore, rvec_fore, lvec_arm, rvec_arm, lvec_clavicle, rvec_clavicle, vec_neck, vec_spine, lvec_femur, rvec_femur, lvec_upleg, rvec_upleg, lvec_lowleg, rvec_lowleg = vectors
        if np.mod(frame+1,100) == 0:
            print('%i frames done. %s seconds.' % (int((frame+1)/100)*100,time.time()-start))
            start=time.time()

        for joint,egoindex in zip([rhand, lhand], range(2)):
            #Get the ego coordinates of the mocap animation joint
            # aux_jointname = skeletonmap.getmatchingjoint(joint.name, sourceanim).name
            ego = EgocentricCoordinate.egolist[egoindex].getCoordFrame(frame)
            currentJointSurfaceNormal = extremityNormal(animation, joint, frame)

#            if frame==170:
#                print('Current Joint Surface Normal:')
#                print(currentJointSurfaceNormal)
#                print('Components Surface Normal:')

            newJointSurfaceNormals = []
            for i in range(len(bodymesh)+len(headmesh)):
                if i<len(headmesh):
                    _, componentSurfaceNormal = mathutils.getCentroid(headmesh[i][0].getPosition(animation, frame),headmesh[i][1].getPosition(animation, frame), headmesh[i][2].getPosition(animation, frame))
                else:
                    j = i-len(headmesh)
                    _, componentSurfaceNormal = mathutils.getCentroid(bodymesh[j][0].getPosition(animation, frame),bodymesh[j][1].getPosition(animation, frame), bodymesh[j][2].getPosition(animation, frame))

                #Get the axis of rotation to align the component surface normal
                axis = np.cross(componentSurfaceNormal,currentJointSurfaceNormal)
                axis_norm = axis/np.linalg.norm(axis)
                #Rotate the component surface normal and get a joint surface normal regarding that component
                matrix = mathutils.matrixRotation(ego.angle[i]*180/np.pi, axis_norm[0],axis_norm[1],axis_norm[2], shape=3)
                newJointSurfaceNormals.append(np.dot(matrix, componentSurfaceNormal))

#                if frame==170:
#                    print(newJointSurfaceNormals[-1])

            for values in DenormEgoLimb(joint, animation, surface, frame, vectors, ego, i+1):
                _, _, _, componentSurfaceNormal = values
                i = i+1
                #Get the axis of rotation to align the component surface normal
                axis = np.cross(componentSurfaceNormal,currentJointSurfaceNormal)
                axis_norm = axis/np.linalg.norm(axis)
                #Rotate the component surface normal and get a joint surface normal regarding that component
                matrix = mathutils.matrixRotation(ego.angle[i]*180/np.pi, axis_norm[0],axis_norm[1],axis_norm[2], shape=3)
                newJointSurfaceNormals.append(np.dot(matrix, componentSurfaceNormal))
            if joint == rfoot or joint == lfoot:
                #Handle foot contact
                componentSurfaceNormal = [0,1,0]
                #Get the axis of rotation to align the component surface normal
                axis = np.cross(componentSurfaceNormal,currentJointSurfaceNormal)
                axis_norm = axis/np.linalg.norm(axis)
                #Rotate the component surface normal and get a joint surface normal regarding that component
                matrix = mathutils.matrixRotation(ego.angle[-1]*180/np.pi, axis_norm[0],axis_norm[1],axis_norm[2], shape=3)
                newJointSurfaceNormals.append(np.dot(matrix, componentSurfaceNormal))

#            if frame == 170:
#                print('Soma:')
#                print((np.asarray(newJointSurfaceNormals)*ego.importance[:,None]).sum(axis=0))

            #Get the mean of the new joint surface  normals
            newJointSurfaceNormal = (np.asarray(newJointSurfaceNormals)*ego.importance[:,None]).sum(axis=0)
            #Get the matrix to rotate the current joint surface normal to the new one
            matrix = mathutils.alignVectors(currentJointSurfaceNormal, newJointSurfaceNormal)
            #Apply this rotation to the joint:
            #Get global rotation matrix
            glbRotationMat = mathutils.shape4ToShape3(joint.getGlobalTransform(frame))
            #Rotate joint
            newGblRotationMat = np.dot(matrix, glbRotationMat)
            #Get new local rotation matrix
            parentGblRotationMat = mathutils.shape4ToShape3(joint.parent.getGlobalTransform(frame))
            newLclRotationMat = np.dot(parentGblRotationMat.T, newGblRotationMat)
            #Get new local rotation euler angles
            newAngle, warning = mathutils.eulerFromMatrix(newLclRotationMat, joint.order)
            joint.rotation[frame] = newAngle[:]
