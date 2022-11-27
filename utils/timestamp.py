import numpy as np


def match_timestamp(ts_list, input_ts, max_offset=None):
    # make sure timestamps have same length
    ts_list_new = [int(ts[:len(input_ts)]) for ts in ts_list]
    ts_list_new = np.array(ts_list_new)
    min_idx = np.argmin(np.abs(ts_list_new - int(input_ts)))

    if max_offset is not None:
        if (ts_list_new[min_idx] - int(input_ts)) < max_offset:
            return ts_list[min_idx]
        else:
            return None

    return ts_list[min_idx]
