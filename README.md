# gifu_drone_node
本パッケージはMAVROSでROSと接続したドローンを制御するためのプログラムです．

## 関数
### void GifuDroneMem::starting()
FCUをOFFBOARDモードに変更しドローンを起動します．\
安全のためにプロポのスイッチ切り替えが必要なように設定しています．\
Ch. 7がアームのスイッチとして設定してあります．\
シミュレーションの場合は自動的にドローンが起動します．

### void GifuDroneMem::shutdown()
FCUに0目標を入力し正常終了します．

### void GifuDroneMem::updateErr()
目標に対する現在位置姿勢の誤差を更新します．

### void GifuDroneMem::publishCtrlInput()
目標姿勢とスロットルを誤差から算出し配信します．

### bool loadParamXML()
パラメータファイルを読み込みます．

### function.cpp .hpp
計算に使用する関数が記述されています．

### feedback.cpp .hpp
PIDフィードバックに関する関数が記述されています．

## パラメータ
gifu_drone_param.xmlに制御パラメータを記述します．\
システム設定と位置方向のPIDパラメータが用意されています．\
パッケージの最上部に配置したパラメータファイルをインポートするように設定しています．\
./src/gifu_drone/gifu_drone_param.xml\
ワークスペースの最上部でrosrunをする必要があります．

simulation\
    true:   シミュレーション\
    false:  実機を使用\
throttle_upper_limit: スロットル値の上限 (下限< <=1)\
throttle_lower_limit: スロットル値の下限 (0<= <上限)\
rc_accept\
    true:   プロポを使用
    false:  プロポを不使用（安全のために基本的にはtrueにすること）
rc_direct_roll\
    true:   roll (y位置)の制御をプロポから行う\
    false:  roll (y位置)の制御を自律で行う\
rc_direct_pitch\
    true:   pitch (x位置)の制御をプロポから行う\
    false:  pitch (x位置)の制御を自律で行う


## SubscriberとPublisher
Topicが/mavrosのものは編集しないで使用可能です．\
Topibが/gdpのものは適宜編集が必要です．\
本パッケージの内容subscriber及びcall backを編集する，または，\
センサ出力のTopicとMessage Typeを本パッケージに合わせてください．

### Subscriber
state_sub: FCUの状態の購読\
    Message Type: mavros_msgs::State\
    Topic:        "mavros/state"\
    Call Back:    state_cb

target_point_sub: 目標位置の購読\
    Message Type: geometry_msgs::Point\
    Topic:        /gdp/target_point\
    Call Back:    target_point_cb

target_yaw_sub: 目標方向の購読\
    Message Type: std_msgs::Float64\
    Topic:        /gdp/target_yaw\
    Call Back:    target_yaw_cb

quit_sub: 停止コマンドの購読\
    Message Type: std_msgs::Empty\
    Topic:        /gdp/quit\
    Call Back:    quit_cb

rc_sub: プロポ入力の購読\
    Message Type: mavros_msgs::RCIn\
    Topic:        /mavros/rc/in\
    Call Back:    rc_cb

battery_sub: バッテリー残量の購読\
    Message Type: sensor_msgs::BatteryState\
    Topic:        /mavros/battery\
    Call Back:    battery_cb

position_sub: 現在位置の購読\
    Message Type: geometry_msgs::PoseStamped\
    Topic:        /gdp/position\
    Call Back:    position_cb

altitude_sub: 現在高度の購読\
    Message Type: std_msgs::Float64\
    Topic:        /gdp/altitude\
    Call Back:    altitude_cb

attitude_sub: 現在姿勢の購読\
    Message Type: sensor_msgs::Imu\
    Topic:        /mavros/imu/data\
    Call Back:    attitude_cb

pose_sub: FCUが保持している現在位置姿勢の購読，シミュレータのみで使用\
    Message Type: geometry_msgs::PoseStamped\
    Topic:        /mavros/local_position/pose\
    Call Back:    pose_cb

### PublisherとClient
reset_init_pub: 他のセンサの初期化に使用可能なTopicの配信\
    Message Type: std_msgs::Bool\
    Topic:        "/gdp/reset_initial"

target_pub: 目標姿勢とスロットル値の配信\
    Message Type: mavros_msgs::AttitudeTarget\
    Topic:        "mavros/setpoint_raw/attitude"

arming_client: FCUをアーム（アイドリングスタート）するためのクライアント\
    Message Type: mavros_msgs::CommandBool\
    Topic:        "mavros/cmd/arming"

set_mode_client: FCUのモードを変更するためのクライアント\
    Message Type: mavros_msgs::SetMode\
    Topic:        "mavros/set_mode"




