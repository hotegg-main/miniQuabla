# Outline
Pythonで記述されたQuablaの簡易版

# Motivation
* 気軽にロケットの飛翔シミュレータを作ってみたい
* モデルをどこまで簡略化できるかの検証

# Usage

```
$ python simulator.py
```

# Set up

必要なPythonのライブラリのインんストール
```
$ python -m pip install -r requirements.txt
```

# Future Work
<!-- * 磁気偏角 -->
* 2段分離
* ペイロード
* 空力パラメータのマッハ数補間
* 上空風ファイルの入力

# Assumption
* 重力加速度:一定
* 質量変化：推力に合わせて変化
    * 一定に比べて、サンプルケースで20mぐらい変わる
* 重心変化：変化率一定