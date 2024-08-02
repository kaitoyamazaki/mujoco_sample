# MuJoCoのサンプルプログラム(C++)

C++を利用したMuJoCoのサンプルプログラムです.

# 結果

サンプルプログラムを実行すると, 下記のようになります.

![実行結果](img/MuJoCo_サンプルプログラム動画_床追加_編集後.gif)

# 環境

サンプルプログラムは以下環境で実行されました.

- ubuntu 22.04LTS
- g++ (Ubuntu 11.4.0-1ubuntu1~22.04) 11.4.0
- MuJoCo 3.2.0

# 事前準備

サンプルプログラムを実行する前に以下の操作が必要です.

## MuJoCoのダウンロード

以下リンクからダウンロードしてください.</br>
[MuJoCo ダウンロードサイト](https://github.com/google-deepmind/mujoco/releases)

## 必要なツールのインストール

`build-essential`と`cmake`, `GLFW`が必要です.
以下コマンドを実行してください.

```
sudo apt update -y
sudo apt install -y build-essential cmake libglfw3-dev
```

## 環境変数の設定

MuJoCoをC++で実行させるために以下操作が必要です.
この操作は[MuJoCo](https://github.com/google-deepmind/mujoco/releases)のダウンロードを前提とします.

```
cd ~/Download
tar -xvzf mujoco-3.2.0linux-x86-64.tar.gz
mkdir ~/.mujoco
mv mujoco-3.2.0 ~/.mujoco
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/.mujoco/mujoco-3.2.0/bin' >> ~/.bashrc
source ~/.bashrc
```

# 実行手順

以下の手順で実行可能です

```
git clone https://github.com/kaitoyamazaki/mujoco_sample.git
cd mujoco_sample
chmod +x make.bash
./make.bash
```

# ドキュメント

MuJoCoには[公式ドキュメント](https://mujoco.readthedocs.io/en/stable/overview.html)があります. ドキュメントを参考にすることでより詳細な機能を知ることができます.

# 参考文献

1. [MuJoCo公式ドキュメント](https://mujoco.readthedocs.io/en/stable/overview.html)
2. [[Python] 物理エンジンMuJoCoの紹介&MJCFドキュメント [MuJoCoチュートリアル①]](https://qiita.com/Yayoi-Habami/items/1bf5a3e05b1516a90381)
3. [[Python] MuJoCo/actuatorドキュメント [MuJoCoチュートリアル②]](https://qiita.com/Yayoi-Habami/items/90f42ea10a32eb20fde8) 