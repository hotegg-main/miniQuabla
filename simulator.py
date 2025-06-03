if __name__=='__main__':

    from solver import run_single, run_loop

    cond = dict()
    cond['Speed']   = {}
    cond['Azimuth'] = {}
    #################################################################
    # USER INPUT
    #################################################################
    path_config = 'example/rocket_config.csv'
    path_result = 'test'
    # 落下分散の風向・風速条件
    #################################################################
    cond['Speed']['min']   = 1.     # 最小風速
    cond['Speed']['step']  = 1.     # 何風速刻みで計算するか
    cond['Speed']['num']   = 7      # 何風速分計算するか
    cond['Azimuth']['min'] = 0.     # 風向の最小値[deg]
    cond['Azimuth']['num'] = 8      # 何風向分計算するか
    #################################################################

    print('Hello World!')

    #################################################################
    # Single Condition Simulation
    #################################################################
    # run_single(path_config, path_result)
    run_loop(path_config, path_result, cond)