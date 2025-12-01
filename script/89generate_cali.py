import numpy as np
from scipy.spatial.transform import Rotation as R
from enum import Enum, auto
from ctypes import *
import sys
import ruamel.yaml

ruamelyaml = ruamel.yaml.YAML()
ruamelyaml.default_flow_style = None
np.set_printoptions(suppress =True)

def inv_T(T):
    # 求逆操作并保持左下三个元素为0
    R = T[:3, :3]
    t = T[:3, 3]
    R_inv = np.linalg.inv(R)
    t_inv = np.dot(-R_inv, t)
    T_inv = np.mat(np.zeros((4, 4)))
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = t_inv
    T_inv[3, 3] = 1
    return T_inv

def getcam0_Tcn_cm1(datain, cam_index_max):
    Tcam0_cncnm1 = np.mat(np.eye(4), dtype=np.float64)
    for i in range(1, cam_index_max+1):
        T_cn_cnm1 = np.mat(np.resize(np.array(datain['cam'+str(i)]['T_cn_cnm1'], dtype=np.float64), (4, 4)))
        Tcam0_cncnm1 = Tcam0_cncnm1.dot(inv_T(T_cn_cnm1))
    print("Tcam0_cncnm1=")
    print(Tcam0_cncnm1)
    return Tcam0_cncnm1
    # Rt0 = np.mat(np.resize(np.array(cam0T_cam_imu), (4, 4)))
    # datain[camright_namespace]['T_cam_imu']

def get_T_cam_imu(datain, cam_index):
    if 'cam' + str(cam_index) in datain:
        if 'T_cam_imu' in datain['cam'+str(cam_index)]:
            T_cam_imu = datain['cam'+str(cam_index)]['T_cam_imu']
            Rt = np.mat(np.resize(np.array(T_cam_imu, dtype=np.float64), (4, 4)))
            return Rt
        else :
            if 'cam' + str(cam_index - 1) in datain:
                Rt_pre = get_T_cam_imu(datain, cam_index - 1)
                T_cn_cnm1 = np.mat(np.resize(np.array(datain['cam'+str(cam_index)]['T_cn_cnm1'], dtype=np.float64), (4, 4)))
                Rt = T_cn_cnm1.dot(Rt_pre)
                return Rt
    print("error")
    return np.mat(np.eye(4), dtype=np.float64)

# 定义枚举类型
class CameraModels(Enum):
    CAMERA_MODEL_NONE = auto()
    CAMERA_MODEL_DISTORTEDPINHOLE = auto()  # pinhole-radtan
    CAMERA_MODEL_EQUIDISTANTPINHOLE = auto()  # pinhole-equidistant
    CAMERA_MODEL_FOVPINHOLE = auto()  # pinhole-fov
    CAMERA_MODEL_OMNI = auto()  # omni-none
    CAMERA_MODEL_DISTORTEDOMNI = auto()  # omni-radtan
    CAMERA_MODEL_EXTENDEDUNIFIED = auto()  # eucm-none
    CAMERA_MODEL_DOUBLESPHERE = auto()  # ds-none
    CAMERA_MODEL_OTHER = auto()

class ParamHeader(Structure):
    _fields_ = [
        ("reserve", c_uint64),
        ("sec", c_uint64),
        ("nsec", c_uint64),
        ("seq", c_uint32),
        ("reserve2", c_uint32),
    ]
    _pack_ = 1

# 定义相机校准结构体
class ParamCamCali(Structure):
    _fields_ = [
        ("cam_id", c_uint8),
        ("camera_model", c_uint32),
        ("T_cam_imu_se3_qw", c_double),
        ("T_cam_imu_se3_qx", c_double),
        ("T_cam_imu_se3_qy", c_double),
        ("T_cam_imu_se3_qz", c_double),
        ("T_cam_imu_se3_x", c_double),
        ("T_cam_imu_se3_y", c_double),
        ("T_cam_imu_se3_z", c_double),
        ("T_cn_cnm1_se3_qw", c_double),
        ("T_cn_cnm1_se3_qx", c_double),
        ("T_cn_cnm1_se3_qy", c_double),
        ("T_cn_cnm1_se3_qz", c_double),
        ("T_cn_cnm1_se3_x", c_double),
        ("T_cn_cnm1_se3_y", c_double),
        ("T_cn_cnm1_se3_z", c_double),
        ("distortion_coeffs_k1", c_double),
        ("distortion_coeffs_k2", c_double),
        ("distortion_coeffs_p1", c_double),
        ("distortion_coeffs_p2", c_double),
        ("distortion_coeffs_k3", c_double),
        ("intrinsics_xi", c_double),
        ("intrinsics_fx", c_double),
        ("intrinsics_fy", c_double),
        ("intrinsics_cx", c_double),
        ("intrinsics_cy", c_double),
        ("resolution_width", c_uint32),
        ("resolution_height", c_uint32),
    ]
    _pack_ = 1

# 定义设备校准结构体
class ParamDevCali(Structure):
    _fields_ = [
        # 假设 param_header_t 是一个已经定义的结构体，这里需要替换为实际的定义
        ("header", ParamHeader),  # 示例：用64字节的字符数组代替
        ("cali_flag", c_uint64),
        ("fx", c_uint32),
        ("fy", c_uint32),
        ("cam", ParamCamCali * 4),
    ]
    _pack_ = 1

# 定义传感器结构体
class ParamSensorCustom(Structure):
    _fields_ = [
        ("header", ParamHeader),  # 示例：用64字节的字符数组代替
        ("valid_flag", c_uint64),
        ("paddle1", c_double * 13),
        ("angular_velocity_x", c_double),
        ("angular_velocity_y", c_double),
        ("angular_velocity_z", c_double),
        ("paddle2", c_double * 9),
        ("linear_acceleration_x", c_double),
        ("linear_acceleration_y", c_double),
        ("linear_acceleration_z", c_double),
    ]
    _pack_ = 1

# 定义设备参数联合体
class DevParamData(Union):
    _fields_ = [
        ("test", c_uint64),
        ("cali", ParamDevCali),
        ("sensor_custom", ParamSensorCustom),
        ("pad", c_uint8*1688)
    ]
    _pack_ = 1
    # _align_ = 1

# 定义设备参数结构体
class DeviceParam(Structure):
    _fields_ = [
        # 假设 dev_param_type_t 是一个枚举或整数类型，这里需要替换为实际的定义
        ("protocol_version", c_uint8),  # 示例：用整数代替
        ("protocol_type", c_uint8),  # 示例：用整数代替
        ("reverse1", c_uint8),  # 示例：用整数代替
        ("reverse2", c_uint8),  # 示例：用整数代替
        ("type", c_uint32),  # 示例：用整数代替
        # ("resver1", c_uint32),
        ("param", DevParamData),
    ]
    _pack_ = 1

def rotation_matrix_to_quaternion(matrix):
    """将4x4齐次矩阵转换为四元数和平移向量"""
    rot = matrix[:3, :3]
    t = matrix[:3, 3]
    r = R.from_matrix(rot)
    quat = r.as_quat()  # 返回格式 [x, y, z, w]
    return quat[3], quat[0], quat[1], quat[2], t[0], t[1], t[2]

def set_camera_info(cam_data, param_cam):
    """将字典数据填充到ParamCamCali结构体"""
    # 设置相机模型
    model_mapping = {
        "pinhole": 0,
        "omni": 4
    }
    param_cam.camera_model = model_mapping.get(cam_data["camera_model"], 0)
    
    # 转换T_cam_imu矩阵到四元数+平移
    T_cam_imu = np.array(cam_data["T_cam_imu"])
    qw, qx, qy, qz, x, y, z = rotation_matrix_to_quaternion(T_cam_imu)
    param_cam.T_cam_imu_se3_qw = qw
    param_cam.T_cam_imu_se3_qx = qx
    param_cam.T_cam_imu_se3_qy = qy
    param_cam.T_cam_imu_se3_qz = qz
    param_cam.T_cam_imu_se3_x = x
    param_cam.T_cam_imu_se3_y = y
    param_cam.T_cam_imu_se3_z = z
    
    # 转换T_cn_cnm1矩阵到四元数+平移
    T_cn_cnm1 = np.array(cam_data["T_cn_cnm1"])
    qw, qx, qy, qz, x, y, z = rotation_matrix_to_quaternion(T_cn_cnm1)
    param_cam.T_cn_cnm1_se3_qw = qw
    param_cam.T_cn_cnm1_se3_qx = qx
    param_cam.T_cn_cnm1_se3_qy = qy
    param_cam.T_cn_cnm1_se3_qz = qz
    param_cam.T_cn_cnm1_se3_x = x
    param_cam.T_cn_cnm1_se3_y = y
    param_cam.T_cn_cnm1_se3_z = z
    
    # 畸变参数
    dist_coeffs = cam_data["distortion_coeffs"]
    param_cam.distortion_coeffs_k1 = dist_coeffs[0]
    param_cam.distortion_coeffs_k2 = dist_coeffs[1]
    param_cam.distortion_coeffs_p1 = dist_coeffs[2]
    param_cam.distortion_coeffs_p2 = dist_coeffs[3]
    param_cam.distortion_coeffs_k3 = dist_coeffs[4] if len(dist_coeffs) >4 else 0.0
    
    # 内参（根据模型类型处理）
    intrinsics = cam_data["intrinsics"]
    if param_cam.camera_model == 4:  # omni模型
        param_cam.intrinsics_xi = intrinsics[0]
        param_cam.intrinsics_fx = intrinsics[1]
        param_cam.intrinsics_fy = intrinsics[2]
        param_cam.intrinsics_cx = intrinsics[3]
        param_cam.intrinsics_cy = intrinsics[4]
    else:  # pinhole模型
        param_cam.intrinsics_fx = intrinsics[0]
        param_cam.intrinsics_fy = intrinsics[1]
        param_cam.intrinsics_cx = intrinsics[2]
        param_cam.intrinsics_cy = intrinsics[3]
    
    # 分辨率
    res = cam_data["resolution"]
    param_cam.resolution_width = res[0]
    param_cam.resolution_height = res[1]
    
    return param_cam

def yaml_to_param_cali(yaml_file):
    """将YAML文件转换为ParamDevCali结构体"""
    with open(yaml_file, 'r') as f:
        data = ruamelyaml.load(f)

    if 'T_cn_cnm1' not in data['cam0']:
        data['cam0']['T_cn_cnm1'] = getcam0_Tcn_cm1(data, 3).tolist()
        # with open('kalibr_cam_chain.yaml', 'w+') as g:
        #     ruamelyaml.dump(data, g)
    if 'T_cam_imu' not in data['cam2']:
        data['cam2']['T_cam_imu'] = get_T_cam_imu(data, 2).tolist()
        # with open('kalibr_cam_chain.yaml', 'w+') as g:
        #     ruamelyaml.dump(data, g)
    if 'T_cam_imu' not in data['cam3']:
        data['cam3']['T_cam_imu'] = get_T_cam_imu(data, 3).tolist()
        # with open('kalibr_cam_chain.yaml', 'w+') as g:
        #     ruamelyaml.dump(data, g)

    param = ParamDevCali()
    
    # 遍历所有摄像头（假设按cam0-cam3顺序排列）
    for i in range(4):
        cam_key = f"cam{i}"
        if cam_key not in data:
            raise ValueError(f"Missing {cam_key} in YAML file")
        
        # 获取对应摄像头结构体
        param_cam = param.cam[i]
        
        # 填充数据
        set_camera_info(data[cam_key], param_cam)
    
    # 设置公共参数头（根据实际需求补充）
    param.header.sec = 0
    param.header.nsec = 0
    param.header.seq = 0
    
    return param

def cmd_set_cali_cam(device_param):
    device_param.protocol_version = 1
    device_param.protocol_type = 1
    device_param.type = 1145241863
    byte_array = bytearray(device_param)
    print("cmd_set_cali_cam")
    #dev.write(ep3, byte_array, timeout=500)
    with open('/tmp/cali', 'wb') as file:
        file.write(byte_array)
    print("cmd_set_cali_cam done")

# 使用示例
def main():
    args = sys.argv
    if (len(args) < 2):
        print("input kalibr yaml path!")
        return

    yaml_path = args[1]
    cali_param = yaml_to_param_cali(yaml_path)
    
    # 验证转换结果
    print("Camera 0 parameters:")
    print(f"fx: {cali_param.cam[0].intrinsics_fx}")
    print(f"Model: {cali_param.cam[0].camera_model}")
    print(f"Resolution: {cali_param.cam[0].resolution_width}x{cali_param.cam[0].resolution_height}")
    device_param = DeviceParam()
    device_param.param.cali = cali_param

    cmd_set_cali_cam(device_param)

if __name__ == '__main__':
    main()

