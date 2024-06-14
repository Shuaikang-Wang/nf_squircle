import numpy as np

from ENV.polygon_partition import PolygonPartition


class SquircleEstimation(object):
    def __init__(self, world):
        self.world = world

    def class_lines_in_group(self, all_line_list):
        all_line_groups = []

        for line_i in all_line_list:
            connected_group = self.find_line_in_groups(line_i, all_line_groups)
            if connected_group is not None:
                connected_group.append(line_i)
            else:
                all_line_groups.append([line_i])

        # Merge groups that share common squircle
        merged_groups = []
        all_groups_pre = all_line_groups
        all_groups_suf = all_line_groups
        all_merge_index = []
        for i, group_i in enumerate(all_groups_pre):
            merged_groups_i = group_i
            if i in all_merge_index:
                continue
            for j, group_j in enumerate(all_groups_suf[i + 1:]):
                j = j + i + 1
                if j in all_merge_index:
                    continue
                if any(self.check_line_intersect(line_i, line_j)
                       for line_i in group_i for line_j in group_j):
                    merged_groups_i = merged_groups_i + group_j
                    all_merge_index.append(j)
            unique_merged_groups_i = []
            [unique_merged_groups_i.append(obj) for obj in merged_groups_i if obj not in unique_merged_groups_i]
            merged_groups.append(unique_merged_groups_i)
        return merged_groups

    def find_line_in_groups(self, line, all_groups):
        for group_i in all_groups:
            for line_j in group_i:
                if self.check_line_intersect(line, line_j):
                    return group_i
        return None

    def check_line_intersect(self, line_i, line_j):
        pass

    def estimate(self):
        for ws in self.world.workspace:
            if len(ws) == 1:
                continue
            ws_1 = ws[1]
            if len(ws_1.accumulated_line_list) != 0:
                all_line_list = ws_1.accumulated_line_list
                # poly_part = PolygonPartition(polygon)
                # result = poly_part.polygon_partition()

        for obs in self.world.obstacles:
            is_valid_sphere = False
            for obs_i in obs:
                if obs_i.s < 0.1 and len(obs_i.accumulated_local_points) > 3:
                    is_valid_sphere = True
                    break
            obs_0 = obs[0]
            if is_valid_sphere:
                estimated_radius = 0.4 / len(obs_0.accumulated_local_points)
                estimated_squircle = [obs_0.center,
                                      obs_0.width - estimated_radius * np.random.uniform(0, 1),
                                      obs_0.height - estimated_radius * np.random.uniform(0, 1),
                                      obs_0.theta,
                                      obs_0.s]
                obs_0.estimated_squircle_list = [estimated_squircle]
            else:
                if len(obs_0.accumulated_line_list) != 0:
                    all_line_list = obs_0.accumulated_line_list
