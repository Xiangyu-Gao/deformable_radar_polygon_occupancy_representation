import numpy as np
from polygon.radar_polygon_ISM import update_polygon_loc_radarScene


def del_list_indexes(l, id_to_del):
    somelist = [i for j, i in enumerate(l) if j not in id_to_del]
    return somelist


class vertexNodes:
    def __init__(self, ):
        self.cons_age = []
        self.vel = []
        self.radar_id = []
        self.confScore = []
        self.loc_x = []
        self.loc_y = []
        self.start = []
        self.latest = []

    def addNode(self, loc_x, loc_y, vel, radar_id, confScore, start):
        self.cons_age.append(1)
        self.vel.append(vel)
        self.radar_id.append(radar_id)
        self.confScore.append(confScore)
        self.loc_x.append(loc_x)
        self.loc_y.append(loc_y)
        self.start.append(start)
        self.latest.append(start)

    def removeNode(self, idx):
        if type(idx) is not list:
            del self.cons_age[idx]
            del self.vel[idx]
            del self.radar_id[idx]
            del self.confScore[idx]
            del self.loc_x[idx]
            del self.loc_y[idx]
            del self.start[idx]
            del self.latest[idx]
        else:
            self.cons_age = del_list_indexes(self.cons_age, idx)
            self.vel = del_list_indexes(self.vel, idx)
            self.radar_id = del_list_indexes(self.radar_id, idx)
            self.confScore = del_list_indexes(self.confScore, idx)
            self.loc_x = del_list_indexes(self.loc_x, idx)
            self.loc_y = del_list_indexes(self.loc_y, idx)
            self.start = del_list_indexes(self.start, idx)
            self.latest = del_list_indexes(self.latest, idx)

    def update_nodes(self, cur_idx, vehicle_motion, vehicle_rotate):
        # remove the unassociated ones
        remove_list = []
        for idx, latest in enumerate(self.latest):
            if latest + 1 < cur_idx:
                remove_list.append(idx)
        self.removeNode(remove_list)

        # update the location of new dictionary
        update_polygon_loc_radarScene(self.loc_x, self.loc_y, self.radar_id, vehicle_motion, vehicle_rotate)

