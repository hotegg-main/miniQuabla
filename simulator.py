if __name__=='__main__':

    from solver import run_single, run_loop

    #################################################################
    # USER INPUT
    #################################################################
    path_config = 'example/rocket_config.csv'
    path_result = 'test'
    #################################################################

    print('Hello World!')

    #################################################################
    # Single Condition Simulation
    #################################################################
    # run_single(path_config, path_result)
    run_loop(path_config, path_result)