import os


def main():
    n_loops = 229
    dataset = "/home/slamemix/Data/RobustSLAM/2D/FR079/graph.g2o"
    output_dir = "/home/slamemix/Data/RobustSLAM/2D/FR079/SPOILED_DATA_EXTREME/"
    res_out = "/0"
    
    dir_idx = 10
    perc = 0.1
    for _ in range(10):
        n = int((perc * n_loops) / (1.0 - perc))
        output = output_dir + str(dir_idx) + res_out

        for idx in range(10):
            command_gen = "python3 generateDataset.py -i " + dataset + " -n " + str(n)
            os.system(command_gen)
            print(f'Name of the final file: {output + str(idx) + ".g2o"}')
            command_ren = "mv new.g2o " + output + str(idx) + ".g2o"
            os.system(command_ren)

        dir_idx += 10
        perc += 0.1

    return 


if __name__ == '__main__' :
    main()
