import h5py
import json
import os
import pickle


def read_radar_scene_h5py(filename):
    with h5py.File(filename, "r") as f:
        # List all groups
        a_group_key = list(f.keys())
        # Get the data
        data_all = []
        for key in a_group_key:
            data_all.append(list(f[key]))

    return a_group_key, data_all


def read_radar_scene_json(filename):
    # Opening JSON file
    f = open(filename)
    # returns JSON object as a dictionary
    json_data = json.load(f)
    # Closing file
    f.close()

    return json_data


if __name__ == '__main__':
    root_dir = 'E:/RadarScenes/data'
    name = 'sequence_143'
    key, radar_data = read_radar_scene_h5py(os.path.join(root_dir, name, 'radar_data.h5'))

    scene_data = read_radar_scene_json(os.path.join(root_dir, name, 'scenes.json'))
    seq_name = scene_data["sequence_name"]
    seq_cat = scene_data["category"]
    seq_start_time = scene_data["first_timestamp"]
    seq_end_time = scene_data["last_timestamp"]
    seq_scenes = scene_data["scenes"]
    seq_scene_keys = list(seq_scenes.keys())

    # Store data (serialize)
    with open('scene.pickle', 'wb') as handle:
        pickle.dump([seq_scenes, seq_scene_keys, radar_data], handle)
