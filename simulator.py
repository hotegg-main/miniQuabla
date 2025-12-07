if __name__=='__main__':

    from solver import run_single, run_loop

    cond = dict()
    cond['Speed']   = {}
    cond['Azimuth'] = {}
    
    #################################################################
    # 1. 計算する機体の諸元ファイルを指定する
    #################################################################
    path_config = 'example/rocket_config.csv'
    path_result = 'test'
    name_case   = 'nominal'
    #################################################################

    print('Hello World!')

    #################################################################
    # 2. 計算するシミュレーションのモードで下の関数を使い分ける
    # 　　（コメントアウトを外す）
    #    run_single() --> 速度、位置などの時間履歴を見る
    #    run_loop()   --> 落下分散図を計算する
    #################################################################
    # run_single(path_config, path_result, name_case)
    run_loop(path_config, path_result)