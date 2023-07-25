# gait-control
6脚ロボットSOLの歩容制御プログラム

接地点追従法を用いて動作

組み込みPCと接続することでナビゲーション制御も可能


# Installation
1. レポジトリのクローン　

[FCPライブラリ](https://gitlab.com/fcp-multi-legged-robot-project/fcp_libraries_hosogaya)もサブモジュールとして登録済みなので以下でインストール可能
```
$ git clone --recursive https://gitlab.com/fcp-multi-legged-robot-project/sol/gait-control.git -b main 
```
FCPライブラリは`lib/fcp_libraries_hosogaya`内に配置される．
gitのサブモジュールについては[こちら](https://qiita.com/sotarok/items/0d525e568a6088f6f6bb)を参照

2. PlatformIOのインストール
VScodeの拡張機能でインストール可能
参考サイト: https://qiita.com/JotaroS/items/1930f156aab953194c9a

# Build
1. `platformio.ini`の編集　`[env:teensy3.6]`ではすでに編集済みなので必要ない
   1. fcp_libraries_hosogayaへのパスを追加する．以下を追記
        ```lib_extra_dirs = lib/fcp_ibraries_hosogaya``
   2. rosserial arduino librariesをインストールさせる．以下を追記
        ```lib_deps = frankjoshua/Rosserial Arduino Library@^0.9.1`

2. teensyのHardware Settingのインストール
OSがLinuxの場合[こちら](https://www.pjrc.com/teensy/tutorial.html)のサイトの1番を参考にTeensyのハードウェア設定をPCに読み込ませる．
```
Install Teensy Loader
The Teensy Loader application downloads programs to your Teensy board and lets you run them. When you use Arduino, this will be done automatically, but doing it manually is a simple way to check that your board works.
Just download the Teensy Loader and the LED Blink samples. No special installation is needed, just double click on the Teensy Loader to run it. You should see this window:

# ここから下の部分
If you are using Linux, the 00-teensy.rules file must be copied to /etc/udev/rules.d/ to give non-root users permission to access the Teensy USB device.
```

3. VSCode左下の`Build`or`Upload`でビルドor書き込み

4. 基板のスイッチをオンにすればロボットが動き出す

# Usage
## Navigation walk or not.
ナビゲーションなしでの歩行を行う場合は，`src/main.cpp`の`531`行目をコメントアウトする
```src/main.cpp line 531
   Preparation::Stand(robot, motor, 0);

   Global_vars::communication = ROS; // enable for using rosserial # Comment out here
   Global_vars::use_modification_with_estimated_pose =
```

<!-- # How to use
1. [FCP_libraries_hosogaya](https://gitlab.com/fcp-multi-legged-robot-project/fcp_libraries_hosogaya)のクローン
   * 場所はどこでも大丈夫です.
2. `platformio.ini`の編集
   1. fcp_libraries_hosogayaへのパスを追加する．以下を追記
        ```lib_extra_dirs = <Path to fcp_libraries_hosogaya>```
    2. rosserial arduino librariesをインストールさせる．以下を追記
        ```lib_deps = frankjoshua/Rosserial Arduino Library@^0.9.1```
3. ビルドしてエラーがでなければOK! -->

## Switch RMP function to previous one
* 目標点追従のRMPを過去使用していたものに切り替えられるようにしました(2022/12/5). ~~`main.cpp`の冒頭で`#define ATTRACTOR_TYPE 2`としてくれれば過去のものに切り替わります。~~ `properties.h`内で`ATTRACTOR_TYPE = 2`とすると過去の物に切り替わります．
* `ATTRACTOR_TYPE = 1`では目標点付近でも速度が落ちずにドンッと脚を下ろすような動作になります。パラメータは`include/rmpflow2_param.h`内の`setAttractorType1`を編集してください．
* `ATTRACTOR_TYPE = 2`では目標点に近づくに連れて速度が落ちていき，そっと脚を下ろすような動作になります．パラメータは`include/rmpflow2_param.h`内の`setAttractorType2`を編集してください．

# Trouble Shooting
1. fcp_libraries_hosogayaのブランチ，バージョンによってはうまく行かないです．fcp_libraries_hosogayaを`main`ブランチの最新バージョンにして下さい．それでもだめでしたら相談して下さい.

サブモジュールのアップデートについては[こちら](https://qiita.com/sotarok/items/0d525e568a6088f6f6bb)を参考に



# Reference
* [Dynamixel_Control_Table](https://www.besttechnology.co.jp/modules/knowledge/?MX%20Series%20Control%20table%282.0%29)
