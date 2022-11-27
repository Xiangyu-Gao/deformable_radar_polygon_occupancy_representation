
"""sensors = {'radar_1': {'id': 1, 'x': 3.663, 'y': -0.873, 'yaw': -1.48418552},
               'radar_2': {'id': 2, 'x': 3.86, 'y': -0.7, 'yaw': -0.436185662},
               'radar_3': {'id': 3, 'x': 3.86, 'y': 0.7, 'yaw': 0.436},
               'radar_4': {'id': 4, 'x': 3.663, 'y': 0.873, 'yaw': 1.484}}"""

snr_elevation = 30  # make sure the minimum snr is greater or equal than zero

radar_configs = {
    'max_dis': 15,  # meters
    'fov_half': 60,  # degrees
    'duration': 60e-3,  # seconds
}

polygon_configs = {
    'dis_rm_virt': 7.5,  # hyper-param, grid search
    'sampling_reg': 2,
    'height_thres_low': -1.5,
    'height_thres': 5,
    'spike_thres_deg': 10,
    'vel_thres': 0.1,
    'virtual_bound': [-140, 140],  # to line of sight
    'pfa': 1e-3,  # param, random
    'prob_mean': 1.546543,  # param, analyze from distribution
    'prob_var': 0.616058,  # param, analyze from distribution
    'epsilon_1': 1,  # hyper-param, random
    'epsilon_2': 0.5,  # hyper-param for ISM
    'gauss_var': 1.0 / 3,  # param, determined by epsilon_1
    'sum_prob_thres': 0.5,  # hyper-param, grid search
    'conf_pen': 1,  # hyper-param for ISM
    'min_age': 2,   # minimum age for a vertex
}

# location of 2 front radars
init_radar_len = 3.663
loc_radar1 = [-0.873, 0, -85]  # upleft radar, px (in m), py and orientation angle (in degrees to horizon)
loc_radar2 = [-0.7, 0.2, -25]  # upmiddle_left radar, px (in m), py and orientation angle (in degrees to horizon)

# location of 2 rear radars
loc_radar3 = [0.7, 0.2, 25]   # upmiddle_right radar, px (in m), py and orientation angle (in degrees to horizon)
loc_radar4 = [0.873, 0, 85]  # upright radar, px (in m), py and orientation angle (in degrees to horizon)

sensor_locations = [loc_radar1, loc_radar2, loc_radar3, loc_radar4]

car_size = [1.07, 4.95]     # meters

radar_enable = [True, False]  # front radars, rear radars
