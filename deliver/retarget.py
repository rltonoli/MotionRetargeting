# -*- coding: utf-8 -*-
"""
Created on Tue Sep  4 15:12:57 2018

@author: Rodolfo Luis Tonoli
"""

import os
import numpy as np
import pyanimation
import surface
import mathutils
import time
from copy import deepcopy
import egocentriccoord
import ik
########################################################
#TODO: https://web.mit.edu/m-i-t/articles/index_furniss.html
#PRE ANIMAR AVATAR, FAZER CORRESPONDENCIA DE TODAS AS JUNTAS
########################################################


def AlignBones(target, source, headAlign = True, spineAlign = True):
    """
    Align the bones of the target skeleton with the bones of the source skeleton.

    Parameters
    ----------
    target : Animation class object
        Input target skeleton. An Animation object with one frame. The skeleton should be in the T-Pose.
    source : Animation class object
        Input source animation. An Animation object that the target skeleton will copy.
    headAlign : bool, optional
        If this is set to True, the neck bone is aligned. This can cause strange looking pose if the skeletons have very different topologies.
    spineAlign : bool, optional
        If this is set to True, the spine bone is aligned. This can cause strange looking pose if the skeletons have very different topologies.
    """
    #Get skeleton map
    sourcemap = source.getskeletonmap()
    targetmap = target.getskeletonmap()

    #Save the reference TPose. The user can export the target animation with a TPose in the first frame.
    for joint in target.getlistofjoints():
        joint.tposerot = joint.rotation[0]
        joint.tposetrans = joint.translation[0]

    #Expand the number of frames of the avatar animation to match the mocap animation
    target.expandFrames(sourcemap.root.translation.shape[0])

    print('Starting Posture Initialization')
    start=time.time()
    #Adapt pose each frame
    for frame in range(source.frames):
        if np.mod(frame+1,100) == 0:
            print('%i frames done. %s seconds.' % (int((frame+1)/100)*100,time.time()-start))
            start=time.time()

        #Get the Height of the root in the base position (Frame = 0)
        #The source and target animation should be in the TPose
        if frame == 0:
            ground_normal = np.array([0,1,0])
            srcHHips = np.dot(sourcemap.hips.getPosition(0), ground_normal)
            tgtHHips = np.dot(targetmap.hips.getPosition(0), ground_normal)
            ratio = tgtHHips/srcHHips

        #Adjust roots/hips height
        srcPosHips = sourcemap.hips.getPosition(frame)
        srcGroundHips = np.asarray([srcPosHips[0], 0, srcPosHips[2]])
        tgtGroundHips = srcGroundHips*ratio
        srcHHips = np.dot(sourcemap.hips.getPosition(frame), ground_normal)
        tgtHHips = srcHHips*ratio
        targetmap.root.translation[frame] = [0,tgtHHips,0] + tgtGroundHips

        #Align the bones in the first frame because the motion capture actor may not be exactly at TPose
        #That is, the spine could be curved, the arms may not be completly extended, etc.
        if frame == 0:
            #Get bones
            srcBones, tgtBones = [], []
            if spineAlign:
                srcBones.append([sourcemap.hips, sourcemap.spine3])
                tgtBones.append([targetmap.hips, targetmap.spine3])
            if headAlign:
                srcBones.append([sourcemap.neck, sourcemap.head])
                tgtBones.append([targetmap.neck, targetmap.head])
            srcBones = srcBones + [[sourcemap.rarm, sourcemap.rforearm],[sourcemap.larm, sourcemap.lforearm],[sourcemap.rforearm, sourcemap.rhand],[sourcemap.lforearm, sourcemap.lhand],[sourcemap.rupleg, sourcemap.rlowleg],[sourcemap.lupleg, sourcemap.llowleg],[sourcemap.rlowleg, sourcemap.rfoot],[sourcemap.llowleg, sourcemap.lfoot]]
            tgtBones = tgtBones + [[targetmap.rarm, targetmap.rforearm],[targetmap.larm, targetmap.lforearm],[targetmap.rforearm, targetmap.rhand],[targetmap.lforearm, targetmap.lhand],[targetmap.rupleg, targetmap.rlowleg],[targetmap.lupleg, targetmap.llowleg],[targetmap.rlowleg, targetmap.rfoot],[targetmap.llowleg, targetmap.lfoot]]
            for mocapbone, avabone in zip(srcBones,tgtBones):
                #Get source and target global transform and rotation matrices from the start of the bone
                p0 = mocapbone[0].getPosition(0)
                p1 = mocapbone[1].getPosition(0)
                srcDirection = mathutils.unitVector(p1-p0)
                #Get source and target global transform and rotation matrices from the end of the bone
                p0 = avabone[0].getPosition(0)
                p1 = avabone[1].getPosition(0)
                tgtDirection = mathutils.unitVector(p1-p0)
                #Rotation matrix to align the bones
                alignMat = mathutils.alignVectors(tgtDirection, srcDirection)
                #Get new global rotation matrix
                tgtGlbTransformMat = avabone[0].getGlobalTransform(frame)
                tgtGlbRotationMat = mathutils.shape4ToShape3(tgtGlbTransformMat)
                tgtNewGblRotationMat = np.dot(alignMat,tgtGlbRotationMat)
                #Get new local rotation matrix
                if not avabone[0] == targetmap.root: #Does not have a parent, transform is already local
                    tgtParentGblRotationMat = mathutils.shape4ToShape3(avabone[0].parent.getGlobalTransform(frame))
                    tgtNewLclRotationMat = np.dot(tgtParentGblRotationMat.T, tgtNewGblRotationMat)
                else:
                    tgtNewLclRotationMat = tgtNewGblRotationMat[:]
                #Get new local rotation euler angles
                tgtNewLclRotationEuler, warning = mathutils.eulerFromMatrix(tgtNewLclRotationMat, avabone[0].order)
                avabone[0].rotation[frame] = tgtNewLclRotationEuler
        else:
            #For the rest of the motion, apply the transform of the mapped source joints in the mapped target joints
            for tgtJoint, srcJoint in zip(targetmap.getJointsNoRootHips(), sourcemap.getJointsNoRootHips()):
                if tgtJoint is not None and srcJoint is not None:
                    previousframe = frame-1 if frame!= 0 else 0
                    #Get source and target global transform and rotation matrices
                    #Even if frame == 0 the matrices need to be recalculated
                    srcGlbTransformMat = srcJoint.getGlobalTransform(frame)
                    srcGlbRotationMat = mathutils.shape4ToShape3(srcGlbTransformMat)
                    tgtGlbTransformMat = tgtJoint.getGlobalTransform(previousframe)
                    tgtGlbRotationMat = mathutils.shape4ToShape3(tgtGlbTransformMat)
                    #Get previous source global transform and rotation matrices
                    srcPreviousGlbTransformMat = srcJoint.getGlobalTransform(previousframe)
                    srcPreviousGlbRotationMat = mathutils.shape4ToShape3(srcPreviousGlbTransformMat)
                    #Get the transform of the source from the previous frame to the present frame
                    transform = np.dot(srcGlbRotationMat, srcPreviousGlbRotationMat.T)
                    #Apply transform
                    tgtNewGblRotationMat = np.dot(transform, tgtGlbRotationMat)
                    #Get new local rotation matrix
                    tgtParentGblRotationMat = mathutils.shape4ToShape3(tgtJoint.parent.getGlobalTransform(frame))
                    tgtNewLclRotationMat = np.dot(tgtParentGblRotationMat.T, tgtNewGblRotationMat)
                    #Get new local rotation euler angles
                    tgtNewLclRotationEuler, warning = mathutils.eulerFromMatrix(tgtNewLclRotationMat, tgtJoint.order)
                    tgtJoint.rotation[frame] = tgtNewLclRotationEuler[:]
        #Set number of frames and duration of each frame as the source animation
        target.frames = source.frames
        target.frametime = source.frametime


#def checkColisions(animation, surface):
#    lforearm, rforearm = animation.getskeletonmap().lforearm, animation.getskeletonmap().rforearm
#    lhand, rhand = animation.getskeletonmap().lhand, animation.getskeletonmap().rhand
#    llowleg, rlowleg = animation.getskeletonmap().llowleg, animation.getskeletonmap().rlowleg
#    lfoot, rfoot = animation.getskeletonmap().lfoot, animation.getskeletonmap().rfoot
#    forearm_radius = min(surface.getPoint('foreLeft').radius, surface.getPoint('foreRight').radius)
#    arm_radius = min(surface.getPoint('armLeft').radius, surface.getPoint('armRight').radius)
#    shin_radius = min(surface.getPoint('shinLeft').radius, surface.getPoint('shinRight').radius)
#    thight_radius = min(surface.getPoint('thightLeft').radius, surface.getPoint('thightRight').radius)
#    for frame in range(animation.frames):
    # Pega juntas dos membros (as 8)
    # Para cada uma delas:
    #     pega a normal dos componentes
    #     verificar a distancia e a produto interno da posicao com a normal
    #     se for o componente mais proximo e o produto interno for negativo, significa que esta penetrando
    #     calcular a penetracao e empurrar a junta pra fora
    # pegar todas as juntas que sao filhas das mãos e pés, fazer a mesma coisa que de cima





def checkName(name):
    """
    Return True if the file in the path/name provided exists inside current path

    :type name: string
    :param name: Local path/name of the back file
    """
    currentpath = os.path.dirname(os.path.realpath(__file__))
    fullpath = os.path.join(currentpath, name)
    return os.path.isfile(fullpath)



# Generate Surface Calibration ###############################################
#start = time.time()
# surface.GetCalibrationFromBVHS('Superficie\Rodolfo\Frente001_mcp.bvh', 'Superficie\Rodolfo\Cabeca001_mcp.bvh', 'Superficie\Rodolfo\Costas001_mcp.bvh',savefile = True, debugmode = False)
#surface.GetCalibrationFromBVHS('Superficie\Emely2\Frente2_mcp.bvh', 'Superficie\Emely2\Cabeca_mcp.bvh', 'Superficie\Emely2\Costas1_mcp.bvh',True,True)
#surface.GetCalibrationFromBVHS('Superficie\Paula\Frente_mcp.bvh', 'Superficie\Paula\Cabeca_mcp.bvh', 'Superficie\Paula\Costas_mcp.bvh',savefile = True,debugmode = True, minrangewide = 30, minpeakwide=40)
#surface.GetCalibrationFromBVHS('Superficie\Paula\Laura_Frente_mcp.bvh', 'Superficie\Paula\Laura_Cabecas_mcp.bvh', 'Superficie\Laura_Paula\Costas_mcp.bvh',True,True)
#print('Surface Calibration done. %s seconds.' % (time.time()-start))


# Path to Source Animation ###################################################
realpath = os.path.dirname(os.path.realpath(__file__))
#sourceanimations = ['Superficie\Rodolfo\Avo_mcp.bvh','Superficie\Rodolfo\Bisavo_mcp.bvh','Superficie\Rodolfo\Bota001_mcp.bvh','Superficie\Rodolfo\Bronzear_mcp.bvh','Superficie\Rodolfo\Cabeca001_mcp.bvh','Superficie\Rodolfo\Calma_mcp.bvh','Superficie\Rodolfo\Caminhada_mcp.bvh','Superficie\Rodolfo\Casas_mcp.bvh','Superficie\Rodolfo\Costas001_mcp.bvh','Superficie\Rodolfo\Edificio_mcp.bvh','Superficie\Rodolfo\Entender_mcp.bvh','Superficie\Rodolfo\Frente001_mcp.bvh','Superficie\Rodolfo\Macarena_mcp.bvh','Superficie\Rodolfo\Maos1_mcp.bvh','Superficie\Rodolfo\Maos2_mcp.bvh','Superficie\Rodolfo\Palmas1_mcp.bvh','Superficie\Rodolfo\Palmas2_mcp.bvh','Superficie\Rodolfo\Procurar_mcp.bvh','Superficie\Rodolfo\Salto001_mcp.bvh','Superficie\Rodolfo\Sapo_mcp.bvh','Superficie\Rodolfo\Sentar1_mcp.bvh','Superficie\Rodolfo\Sentar2_mcp.bvh','Superficie\Rodolfo\Surdo_mcp.bvh']
sourceanimations = ['Superficie\Rodolfo2\Avo_mcp.bvh','Superficie\Rodolfo2\Bisavo_mcp.bvh', 'Superficie\Rodolfo2\Bota_mcp.bvh','Superficie\Rodolfo2\Bota001_mcp.bvh','Superficie\Rodolfo2\Bronzear_mcp.bvh','Superficie\Rodolfo2\Calma_mcp.bvh','Superficie\Rodolfo2\Caminhar_mcp.bvh','Superficie\Rodolfo2\Casas_mcp.bvh','Superficie\Rodolfo2\Edificio_mcp.bvh','Superficie\Rodolfo2\Entender_mcp.bvh','Superficie\Rodolfo2\Macarena_mcp.bvh','Superficie\Rodolfo2\Maos_mcp.bvh','Superficie\Rodolfo2\Maos001_mcp.bvh','Superficie\Rodolfo2\Palmas_mcp.bvh','Superficie\Rodolfo2\Palmas001_mcp.bvh','Superficie\Rodolfo2\Procurar_mcp.bvh','Superficie\Rodolfo2\Salto_mcp.bvh','Superficie\Rodolfo2\Sapo_mcp.bvh','Superficie\Rodolfo2\Sentar_mcp.bvh','Superficie\Rodolfo2\Sentar001_mcp.bvh','Superficie\Rodolfo2\Surdo_mcp.bvh']
#sourceanimations = ['Superficie\Paula\Avo_mcp.bvh', 'Superficie\Paula\Avo001_mcp.bvh','Superficie\Paula\Bisavo_mcp.bvh', 'Superficie\Paula\Bota_mcp.bvh','Superficie\Paula\Bronzear_mcp.bvh','Superficie\Paula\Cabeca_mcp.bvh','Superficie\Paula\Costas_mcp.bvh','Superficie\Paula\Danca_mcp.bvh','Superficie\Paula\Edificio_mcp.bvh','Superficie\Paula\Entender_mcp.bvh', 'Superficie\Paula\Frente_mcp.bvh','Superficie\Paula\Macarena_mcp.bvh','Superficie\Paula\Palmas_mcp.bvh','Superficie\Paula\Polichinelo_mcp.bvh','Superficie\Paula\Pulo_mcp.bvh','Superficie\Paula\Sapo_mcp.bvh','Superficie\Paula\Sentar_mcp.bvh','Superficie\Paula\Surdo_mcp.bvh']
# sourceanimations = ['Superficie\Rodolfo2\Procurar_mcp.bvh']
#sourceanimations = ['Superficie\Emely2\RotMaos_mcp.bvh']
#sourceanimations = ['Superficie\Paula\Avo_mcp.bvh']
# sourceanimations = [os.path.join(realpath, sourceanimations[i]) for i in range(len(sourceanimations))]

# Path to Source Calibration #################################################
sourcesurface = os.path.join(realpath, 'Superficie\Rodolfo\surface_rodolfo.txt')
#sourcesurface = os.path.join(realpath, 'Superficie\Emely2\surface_emely2.txt')
#sourcesurface = os.path.join(realpath, 'Superficie\Paula\surface_1.txt')

# Path to Target TPose BVH File ##############################################
# Path to Target Calibration File ############################################
# Path to Target Skeletom Map File ###########################################
# targettpose = os.path.join(realpath, 'TalitaTPose.bvh')
# targetsurface = os.path.join(realpath, 'surface_avatar.csv')
# skeletonmappath = None #Use standard
targettpose = os.path.join(realpath, 'AragorTPose.bvh')
targetsurface = os.path.join(realpath, 'surface_aragor.csv')
skeletonmappath = os.path.join(realpath, 'skeletonmap_aragor.csv')


for path in sourceanimations:
    retargettime = time.time()

    # Surface Calibration #####################################################
    start = time.time()
    mocapSurface = surface.GetMoCapSurfaceFromTXT(sourcesurface, highpolymesh = False)
    print('Surface from file done. %s seconds.' % (time.time()-start))

    #Read mocap bvh file #####################################################
    source_filename = os.path.basename(path)
    start = time.time()
    mocap = pyanimation.ReadFile(path, surfaceinfo=mocapSurface)
    print('MoCap BVH read done. %s seconds.' % (time.time()-start))
    # mocap.PlotPose(0, mocapSurface)
#    mocap.PlotAnimation()

    #Read TPose bvh file #####################################################
    start = time.time()
    avatar = pyanimation.ReadFile(targettpose)
    #Get the avatar surface data
    avatarSurface = surface.GetAvatarSurfaceFromCSV(targetsurface, highpolymesh = False)
    #Scale the avatar surface data accordingly to the TPose bvh data
    print('Avatar BVH read done. %s seconds.' % (time.time()-start))

    # avatar.PlotPose(0,avatarSurface)

    # Initialize pose (should be done at each frame?) ########################
    AlignBones(avatar, mocap, getpositions = False, headAlign = True, spineAlign = False)
    #start = time.time()
    #print('Starting avatar surface position estimation')
    #surface.AvatarSurfacePositionEstimation(avatar, avatarSurface)
    #print('Done. %s seconds.' % (time.time()-start))

#    avatar.PlotPose(400, avatarSurface)
#    mocap.PlotAnimation(mocapSurface)

    # Calculate egocentric coordinates ############################################
    start = time.time()
    print('Getting Egocentric coordinates')
    #egocoord, targets = GetEgocentricCoordinates(mocap, mocapSurface, avatar, avatarSurface)
    egocoord = egocentriccoord.GetEgocentricCoordinatesTargets(mocap, mocapSurface, avatar, avatarSurface)
    print('Egocentric coordinates done. %s seconds.' % (time.time()-start))


    # Aplica Cinemática Inversa ###################################################
    targetRHand = [0,0,0]
    targetLHand = [0,0,0]

    JacRHand = ik.SimpleJacobian(avatar, avatar.getskeletonmap().rhand, depth = 5)
    JacLHand = ik.SimpleJacobian(avatar, avatar.getskeletonmap().lhand, depth = 5)
    iklogRHand = []
    iklogLHand = []
    start=time.time()
    print('Starting IK')
    for frame in range(avatar.frames):
        targetRHand = egocoord[0].getTarget(frame)
        targetLHand = egocoord[1].getTarget(frame)
        logR = JacRHand.jacobianTranspose(frame=frame, target=targetRHand)
        logL = JacLHand.jacobianTranspose(frame=frame, target=targetLHand)
        iklogRHand.append(logR)
        iklogLHand.append(logL)
        targetRHand = [0,0,0]
        targetLHand = [0,0,0]
        if np.mod(frame+1,100) == 0:
            print('%i frames done. %s seconds.' % (int((frame+1)/100)*100,time.time()-start))
            start=time.time()


    # # Adjust Limb Extremities ##################################################
    egocentriccoord.AdjustExtremityOrientation(avatar, avatarSurface, egocoord, mocap)


    # # Save File ###################################################
    output_filename = source_filename[:-4] + '_retarget'
    of_aux = output_filename
    i = 0
    while checkName(output_filename+'.bvh'):
        i = i + 1
        output_filename = of_aux + str(i)

    pyanimation.WriteBVH(avatar, realpath, output_filename,refTPose=True)

    print('Done! Total time: %s seconds.' % (time.time()-retargettime))



    #avatar.checkPose()

    #plotanimation.AnimPlotBones(listofjoints[0].GetBones())


    #surfacepoints = []
    #for point in listofpoints:
    #   if len(point.position)>0:
    #       surfacepoints.append(point)

    #plotanimation.AnimationSurface(listofjoints[0].GetBones(), surfacepoints)
    #plotanimation.AnimPlotBones(listofjoints_avatar[0].GetBones())
    #plotanimation.AnimationSurfaceAndVectors(listofjoints[0].GetBones(), surfacepoints, toPlot)

    #avatar.PlotAnimation(avatarSurface)
    #avatar.PlotAnimation()

    #



# PARA GERAR OS GRÁFICOS: TODOS OS TARGETS
#targetRHandlist = []
#targetLHandlist = []
#targetRHand = [0,0,0]
#targetLHand = [0,0,0]
#for frame in range(avatar.frames):
#    for i in range(len(egocoord[0].importance)):
#        targetRHand = targetRHand + egocoord[2*frame].importance[i]*targets[2*frame][i]
#        targetLHand = targetLHand + egocoord[2*frame+1].importance[i]*targets[2*frame+1][i]
#    targetRHandlist.append(targetRHand[:])
#    targetLHandlist.append(targetLHand[:])
#    targetRHand = [0,0,0]
#    targetLHand = [0,0,0]
#LHtarget = []
#RHtarget = []
#LHee = []
#RHee = []
#LH = mocap.getJoint('LeftHand')
#RH = mocap.getJoint('RightHand')
#for frame in range(len(targetLHandlist)):
#    LHtarget.append(np.linalg.norm(targetLHandlist[frame]))
#    RHtarget.append(np.linalg.norm(targetRHandlist[frame]))
#    LHee.append(np.linalg.norm(LH.getPosition(frame)))
#    RHee.append(np.linalg.norm(RH.getPosition(frame)))

#r_headimportancerighthand = []
#r_bodyimportancerighthand = []
#r_headproxirighthand = []
#r_bodyproxirighthand = []
#r_headorthorighthand = []
#r_bodyorthorighthand = []
#l_headimportancerighthand = []
#l_bodyimportancerighthand = []
#l_headproxirighthand = []
#l_bodyproxirighthand = []
#l_headorthorighthand = []
#l_bodyorthorighthand = []
#for i in range(len(egocoord)):
#    if egocoord[i].joint.name == 'RightHand':
#        r_headimportancerighthand.append(np.mean(egocoord[i].importance[0:21]))
#        r_headproxirighthand.append(np.mean(egocoord[i].proxi[0:21]))
#        r_headorthorighthand.append(np.mean(egocoord[i].ortho[0:21]))
#        r_bodyimportancerighthand.append(np.mean(egocoord[i].importance[21:]))
#        r_bodyproxirighthand.append(np.mean(egocoord[i].proxi[21:]))
#        r_bodyorthorighthand.append(np.mean(egocoord[i].ortho[21:]))
#    else:
#        l_headimportancerighthand.append(np.mean(egocoord[i].importance[0:21]))
#        l_headproxirighthand.append(np.mean(egocoord[i].proxi[0:21]))
#        l_headorthorighthand.append(np.mean(egocoord[i].ortho[0:21]))
#        l_bodyimportancerighthand.append(np.mean(egocoord[i].importance[21:]))
#        l_bodyproxirighthand.append(np.mean(egocoord[i].proxi[21:]))
#        l_bodyorthorighthand.append(np.mean(egocoord[i].ortho[21:]))

#lista = egocoord[0].framecoord[260].ortho
#for point,i in zip(lista, range(len(lista))):
#    print("%i: %f" %(i,point))
