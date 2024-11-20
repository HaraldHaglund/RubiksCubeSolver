import pickle
import numpy as np
import os

def saveLog(log_dict, final_iteration, args):
    # shave off the zeros, noone needs 'em
    for key in log_dict:
        log_dict[key] = log_dict[key][:final_iteration]
    # TODO make generic:
    #  - generate name based on args + add index
    #  - you need to run shell ls to get the index,
    #    there's at least one chalmers project with code for that
    if os.path.exists('./data'):
        run_file_path = "./data/clik_run_001.pickle"
        args_file_path = "./data/clik_run_001_args.pickle"
    else:
        os.makedirs('/tmp/data', exist_ok=True)
        run_file_path = "/tmp/data/clik_run_001.pickle"
        args_file_path = "/tmp/data/clik_run_001_args.pickle"
    # save the logged data
    # you need to open it binary for pickling
    log_file = open(run_file_path, 'wb')
    pickle.dump(log_dict, log_file)
    log_file.close()
    # but also save the arguments
    # pretty sure you need to open it binary for pickling
    log_file = open(args_file_path, 'wb')
    pickle.dump(args, log_file)
    log_file.close()


def cleanUpRun(log_dict, final_iteration, n_iterations_you_want):
    # shave off the zeros at the end
    for key in log_dict:
        log_dict[key] = log_dict[key][:final_iteration]
    # and now keep only every nth iteration
    # because you don't want to plot too much
    if final_iteration > n_iterations_you_want:
        nth_to_keep = final_iteration // n_iterations_you_want
        bool_array = [i % nth_to_keep == 0 for i in range(final_iteration)]
        # actual final number
        # True is turned to 0, False to 0, praised by python and its ways
        n_iters = np.sum(bool_array)
        for key in log_dict:
            log_dict[key] = log_dict[key][bool_array]

    return log_dict


def loadRunForAnalysis(log_data_file_name, args_file_name):
    log_data_file = open(log_data_file_name, 'rb')
    args_file = open(args_file_name, 'rb')
    log_data = pickle.load(log_data_file)
    args = pickle.load(args_file)
    # if you're analyzing, you're not running anything on the real robot
    args.simulation = True
    args.pinocchio_only = True
    return log_data, args
