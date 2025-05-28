:::mermaid
flowchart TD
start1([スタート])
subgraph Parameter
param_read[入力ファイル読み込み] -->
param_initial[初期値計算]
end
subgraph Solver
solve_launch_clear[ランチクリア計算] -->
solve_trajectory[弾道軌道計算] -->
solve_parachute[パラシュート降下計算]
end
subgraph Result
end
start1 --> param_read
param_initial --> solve_launch_clear
:::

# Parameter
:::mermaid
flowchart TD
start1([スタート]) --> 
read_config[csvファイルの読み込み] --> calc_shape
subgraph Structure
calc_shape[形状寸法] -->
calc_mass[重量] -->
calc_cg[重心] -->
calc_inertia[慣性モーメント]
end
subgraph Engine
read_thrust[推力履歴読み込み]
end
subgraph Aero
end
:::

# Dynamics
:::mermaid
---
title: Dynamics Trajectory
---
flowchart TD
start1([スタート])
end1([終了])
input_param[[基本変数time,pos,vel,quat,omega]]
%% calc_attitude[["(quat)"</br>姿勢角計算]]
calc_dcm[["(quat)"</br>座標変換行列計算]]
get_area[["geometry."</br>代表面積]]
calc_mass[["geometry.(time)"</br>慣性モーメント計算]]
calc_lcg[["params.(time)"</br>重心計算]]
calc_env[["atmos.(pos)"</br>重力加速度、大気密度、（大気圧計算）、音速]]
calc_wind[["atmos.(pos)"</br>風ベクトル計算]]
calc_vel_air[["(vel,wind)"</br>対気速度計算]]
calc_aoa[["(vel_air)"</br>迎角、横滑滑り角計算]]
calc_mach[["(vel_air)"</br>マッハ数、動圧計算]]
calc_aero_param[["params.(mach)"</br>圧力中心、軸力係数、法線力傾斜計算]]
calc_aero_force[["(Q, aoa, coeff, S)"</br>軸力、法線力、横力]]
calc_thrust[["engine.(time)"</br>推力]]
calc_gravity[["(g)"</br>重力計算]]
calc_aero_moment[["(lcg,lcp,Faero)"</br>復元モーメント計算]]
calc_aero_damp_moment[["(Q,omega,vel_air,coeff,S,L,d)"</br>空力減衰モーメント]]
calc_gyro_moment[["(Ij, omega)"</br>ジャイロモーメント]]
calc_acc[["(omega, v, m, Faero, Fthrust, Fg)"</br>加速度]]
calc_omegadot[["(Ij, momGyro, momAero, momAeroDamp)"</br>角加速度]]
calc_quatdot[["(omega)"</br>クォータニオン変化]]
sw_mode{ランチクリア?}
set_no_rotation[[回転なし]]
calc_dxdt[[dx/dt計算]]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
start1 --> input_param
input_param --> calc_dcm -->
calc_mass --> calc_lcg
calc_lcg --> calc_env
calc_env --> calc_wind --> calc_vel_air
calc_vel_air --> calc_aoa --> calc_mach --> calc_aero_param
calc_aero_param --> calc_aero_force
subgraph Froce
    calc_aero_force --> calc_thrust --> calc_gravity
end
%% calc_gravity --> calc_aero_moment
calc_gravity --> sw_mode
sw_mode --YES--> calc_aero_moment
sw_mode --NO--> set_no_rotation --> calc_dxdt
subgraph Moment
    calc_aero_moment --> calc_aero_damp_moment --> calc_gyro_moment
end
calc_gyro_moment --> calc_acc --> calc_omegadot --> calc_quatdot --> calc_dxdt
calc_dxdt --> end1
:::

# Solver


:::mermaid
---
title: Main Rourtine
---
flowchart TD
start1([スタート])
end1([終了])
solve_ODE_trajectory[[ODE求解]]
%% update_var[[基本変数更新]]
set_init_trajectory[[初期値セット]]
save_trajectory_log[[ログ保存]]
calc_event_value_trajectory[[イベント値保存]]
set_init_parachute[[減速落下初期値セット]]
solve_ODE_parachute[[ODE求解]]
save_parachute_log[[ログ保存]]
calc_event_value_parachute[[イベント値保存]]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
start1 --> set_init_trajectory
subgraph trajectory
set_init_trajectory --> solve_ODE_trajectory --> save_trajectory_log --> calc_event_value_trajectory
end
calc_event_value_trajectory --> set_init_parachute
subgraph parachute
set_init_parachute --> solve_ODE_parachute --> save_parachute_log --> calc_event_value_parachute
end
subgraph aa
end
:::

:::mermaid
---
title: Template
---
flowchart TD
start1([スタート])
end1([終了])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
subgraph aa
end
:::