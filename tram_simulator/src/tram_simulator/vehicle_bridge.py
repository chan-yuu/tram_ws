"""Load vehicle model and actuators from tram_description package."""
import os
import sys
import json
import rospkg
import rospy


def _get_desc_path():
    rp = rospkg.RosPack()
    return rp.get_path('tram_description')


def load_vehicle_cfg():
    desc_path = _get_desc_path()
    json_path = os.path.join(desc_path, 'config', 'vehicle.json')
    with open(json_path, 'r') as f:
        vehicle_json = json.load(f)

    def param(name, default):
        if rospy.has_param('~' + name):
            return rospy.get_param('~' + name)
        return rospy.get_param('/vehicle_6state_dyn/' + name, default)

    dyn_overrides = {
        'C_alpha_f': param('C_alpha_f', 180000.0),
        'C_alpha_r': param('C_alpha_r', 180000.0),
        'mu0':       param('mu0', 0.85),
        'c_roll':    param('c_roll', 0.015),
        'cd_a':      param('cd_a', 6.0),
        'rho':       param('rho', 1.225),
        'grade_percent': param('grade_percent', 0.0),
        'eta_drive': param('eta_drive', 0.92),
        'h_cg':      param('h_cg', 1.1),
        'n_drive_motors': param('n_drive_motors', 2),
    }

    return {
        'vehicle_json': vehicle_json,
        'vehicle': {},
        'vehicle_6state_dyn': dyn_overrides,
    }


def build_from_description():
    """Return (FourWSVehicle6StateDyn, VehicleActuators) built from package config."""
    desc_path = _get_desc_path()
    models_path = os.path.join(desc_path, 'models')
    if models_path not in sys.path:
        sys.path.insert(0, models_path)

    from vehicle_6state_dyn import build_vehicle_6state_dyn, build_actuators

    cfg = load_vehicle_cfg()
    vehicle = build_vehicle_6state_dyn(cfg)
    actuators = build_actuators(cfg)
    return vehicle, actuators
