import xml.etree.ElementTree as ET

filexml = '/sdcard/test_case/SLAMRecord/helmet_2022-08-01-15-11-58/device_calibration.xml'


def PrintK(focal_length, principal_point):
    print("K matrix: ")
    focal_lengths = focal_length.split(' ')
    principal_points = principal_point.split(' ')
    print(focal_lengths[0], ', ', 0.0, ', ', principal_points[0], ', ', 0.0, ', ', focal_lengths[1], ', ', principal_points[1], ', ', 0.0, ', ', 0.0, ', ', 1)

def PrintP(focal_length, principal_point, translation):
    print("P matrix: ")
    focal_lengths = focal_length.split(' ')
    principal_points = principal_point.split(' ')
    translations = translation.split(' ')
    print(focal_lengths[0], ', ', 0.0, ', ', principal_points[0], ', ', translations[0], ', ', 0.0, ', ', focal_lengths[1], ', ', principal_points[1], ', ',translations[1], ', ', 0.0, ', ', 0.0, ', ', 1, ', ', translations[2])

def PrintRadioD(radial_distortion):
    print("radial_distortion: ")
    radial_distortions = radial_distortion.split(' ')
    print(radial_distortions[0], ', ', radial_distortions[1], ', ', radial_distortions[2], ', ', radial_distortions[3])

def PrintR(rowMajorRotationMat):
    print('RotationMatrix: ')
    rowMajorRotationMats = rowMajorRotationMat.split(' ')
    r_str = ''
    for ele in rowMajorRotationMats:
        r_str = r_str + ele + ', '
    print(r_str)

def PrintT(translation):
    print("translation: ")
    translations = translation.split(' ')
    t_str = ''
    for ele in translations:
        t_str = t_str + ele + ', '
    print(t_str)


root = ET.parse(filexml).getroot()
for Camera in root.findall('Camera'):
    id = int(Camera.get('id'))
    if (id != 0 and id != 3):
        continue
    print('cam id ', id)
    Calibration = Camera.find("Calibration")
    principal_point = Calibration.get("principal_point")
    # print(principal_point)
    focal_length = Calibration.get("focal_length")
    # print(focal_length)
    radial_distortion = Calibration.get("radial_distortion")
    PrintRadioD(radial_distortion)
    PrintK(focal_length, principal_point)

    Rig = Camera.find("Rig")
    translation = Rig.get("translation")
    # print(translation)
    rowMajorRotationMat = Rig.get("rowMajorRotationMat")
    # print('RotationMatrix ', rowMajorRotationMat)
    PrintR(rowMajorRotationMat)
    PrintP(focal_length, principal_point, translation)
    PrintT(translation)
    print("--------------------------------------------------")




