import smbus
import time
import struct

class BNO055:
	BNO055_ADDRESS_A 				= 0x28        #代替i2cアドレス
	BNO055_ADDRESS_B 				= 0x29        #デフォルトi2cアドレス, 先のアドレスも含め二つのアドレスがあることでハードウェア的な余裕がある(2つのハードを接続できる？)
	BNO055_ID 		 			= 0xA0        #chipIDであり、BNO055に固有の値

	# Power mode settings
	POWER_MODE_NORMAL   				= 0X00        #標準稼働状態normal:0x00をPOWER_MODE_NORMALに割り当てる、注意！！データシートのp55において、大量に書いてある0x00は電源リセット時などにおける初期値であり、pms, omsの意味合いはないということに注意
	POWER_MODE_LOWPOWER 				= 0X01        #低電力稼働状態normal:0x01をPOWER_MODE_LOWPOWERに割り当てる
	POWER_MODE_SUSPEND  				= 0X02        #稼働停止状態normal:0x02をPOWER_MODE_SUSPENDに割り当てる

	# Operation mode settings(# Power mode settingsとは別のレジスタが使われるため、数字の重複が可能である。例えば、0x00はpmsでは標準稼働状態であり、omsではconfigモード)
	OPERATION_MODE_CONFIG 				= 0X00        #configモード、有効にすると出力データはすべて0になる→書き込み可能なレジスタマップのすべてのエントリを変更できる(センサのオフセット値(無入力状態での実際値とのずれ)などの変更が可能)
	OPERATION_MODE_ACCONLY 				= 0X01        #加速度のみ有効化
	OPERATION_MODE_MAGONLY 				= 0X02        #地磁気のみ有効化
	OPERATION_MODE_GYRONLY 				= 0X03        #ジャイロのみ有効化
	OPERATION_MODE_ACCMAG 				= 0X04        #加速度・地磁気を有効化
	OPERATION_MODE_ACCGYRO 				= 0X05        #加速度・ジャイロを有効化
	OPERATION_MODE_MAGGYRO 				= 0X06        #地磁気・ジャイロを有効化
	OPERATION_MODE_AMG 				= 0X07        #加速度・地磁気・ジャイロを有効化
	OPERATION_MODE_IMUPLUS 				= 0X08        #加速度・ジャイロの組み合わせ
	OPERATION_MODE_COMPASS 				= 0X09        #地磁気による方位の測定
	OPERATION_MODE_M4G 				= 0X0A        #回転の検出にジャイロでなく、地磁気を使用
	OPERATION_MODE_NDOF_FMC_OFF 			= 0X0B        #加速度・地磁気・ジャイロの組み合わせ、地磁気の校正なし
	OPERATION_MODE_NDOF 				= 0X0C        #加速度・地磁気・ジャイロの組み合わせ、地磁気の校正あり

	# Output vector type(データシートp60以降参考、測定を開始すると、以下に示したbyteにデータが格納されると考えてよい)
	VECTOR_ACCELEROMETER 				= 0x08        #x軸加速度データ下位8bit(1byte),(重力含む)
	VECTOR_MAGNETOMETER  				= 0x0E        #x軸地磁気データ下位8bit(1byte)
	VECTOR_GYROSCOPE     				= 0x14        #x軸ジャイロデータ下位8bit(1byte)
	VECTOR_EULER         				= 0x1A        #方位データ下位8bit(1byte)
	VECTOR_LINEARACCEL   				= 0x28        #x軸線形加速度データ下位8bit(1byte), 重力を含まない純粋な動きの加速度
	VECTOR_GRAVITY       				= 0x2E        #x軸重力データ下位8bit(1byte),(センサにとっての軸であるため、z軸だけに重力が働くとは考えられない）

	# REGISTER DEFINITION START(BNO055にはページレジスタという概念がある。簡単に言うと0x00;データ取得のレジスタ群が存在するページ0, 0x01;設定・チューニング用のレジスタ群が存在するページ1)
	BNO055_PAGE_ID_ADDR 				= 0X07        #ページIDレジスタに対応。ページ1を使いたいときにはbus.write_byte_data(BNO055_ADDRESS, BNO055_PAGE_ID_ADDR, 0x01)と入力する

	#識別子の割り当て(おもにデバッグや確認として使われる)データシートp60~参照
	BNO055_CHIP_ID_ADDR 				= 0x00        #chip id 機器に固有の番号
	BNO055_ACCEL_REV_ID_ADDR 			= 0x01        #加速度センサの識別子(chip id)
	BNO055_MAG_REV_ID_ADDR 				= 0x02        #地磁気センサの識別子(chip id) 
	BNO055_GYRO_REV_ID_ADDR 			= 0x03        #ジャイロセンサの識別子(chip id)
	BNO055_SW_REV_ID_LSB_ADDR 			= 0x04        #ソフトウェアの識別子下位バイト(chip id)    主にソフトウェアのバージョンを示し、更新が必要かどうかの確認になる
	BNO055_SW_REV_ID_MSB_ADDR 			= 0x05        #ソフトウェアの識別子上位バイト(chip id)　　上と同じ
	BNO055_BL_REV_ID_ADDR 				= 0X06        #加速度センサの識別子(chip id)

	# Accel data register (加速度データ定義)
	BNO055_ACCEL_DATA_X_LSB_ADDR 			= 0X08        #x軸加速度データ下位8bit(1byte)   ovtの1行目と同じ値であるが、用途が違う。先ほどのVECTOR_ACCCEROMETERではx, y, z軸の加速度データの先頭に位置することを利用し、read_vector(VECTOR_ACCELEROMETER)というような関数が6byteで読むように定義されていれば、加速度データを一気に読み取れる
	BNO055_ACCEL_DATA_X_MSB_ADDR 			= 0X09        #x軸加速度データ上位8bit(1byte)
	BNO055_ACCEL_DATA_Y_LSB_ADDR 			= 0X0A        #y軸加速度データ下位8bit(1byte)
	BNO055_ACCEL_DATA_Y_MSB_ADDR 			= 0X0B        #y軸加速度データ上位8bit(1byte)
	BNO055_ACCEL_DATA_Z_LSB_ADDR 			= 0X0C        #z軸加速度データ下位8bit(1byte)
	BNO055_ACCEL_DATA_Z_MSB_ADDR 			= 0X0D        #z軸加速度データ上位8bit(1byte)

	# Mag data register（地磁気データ定義）
	BNO055_MAG_DATA_X_LSB_ADDR 			= 0X0E        #x軸地磁気データ下位8bit(1byte)
	BNO055_MAG_DATA_X_MSB_ADDR 			= 0X0F        #x軸加速度データ上位8bit(1byte)
	BNO055_MAG_DATA_Y_LSB_ADDR 			= 0X10        #y軸地磁気データ下位8bit(1byte)
	BNO055_MAG_DATA_Y_MSB_ADDR 			= 0X11        #y軸地磁気データ上位8bit(1byte)
	BNO055_MAG_DATA_Z_LSB_ADDR 			= 0X12        #z軸地磁気データ下位8bit(1byte)
	BNO055_MAG_DATA_Z_MSB_ADDR			= 0X13        #z軸地磁気データ上位8bit(1byte)

	# Gyro data registers (ジャイロデータ定義)
	BNO055_GYRO_DATA_X_LSB_ADDR 			= 0X14        #同様
	BNO055_GYRO_DATA_X_MSB_ADDR 			= 0X15        #同様
	BNO055_GYRO_DATA_Y_LSB_ADDR 			= 0X16        #同様
	BNO055_GYRO_DATA_Y_MSB_ADDR 			= 0X17        #同様
	BNO055_GYRO_DATA_Z_LSB_ADDR 			= 0X18        #同様
	BNO055_GYRO_DATA_Z_MSB_ADDR 			= 0X19        #同様
	
	# Euler data registers (方位データ定義)
	BNO055_EULER_H_LSB_ADDR 			= 0X1A        #同様
	BNO055_EULER_H_MSB_ADDR 			= 0X1B        #同様
	BNO055_EULER_R_LSB_ADDR 			= 0X1C        #同様
	BNO055_EULER_R_MSB_ADDR 			= 0X1D        #同様
	BNO055_EULER_P_LSB_ADDR 			= 0X1E        #同様
	BNO055_EULER_P_MSB_ADDR 			= 0X1F        #同様

	# Quaternion data registers (姿勢に関するデータ定義)
	BNO055_QUATERNION_DATA_W_LSB_ADDR 		= 0X20        #同様(この関数は(w, x, y, z)で表すためほかのものより定義2行長めです)
	BNO055_QUATERNION_DATA_W_MSB_ADDR 		= 0X21        #同様
	BNO055_QUATERNION_DATA_X_LSB_ADDR 		= 0X22        #同様
	BNO055_QUATERNION_DATA_X_MSB_ADDR 		= 0X23        #同様
	BNO055_QUATERNION_DATA_Y_LSB_ADDR 		= 0X24        #同様
	BNO055_QUATERNION_DATA_Y_MSB_ADDR 		= 0X25        #同様
	BNO055_QUATERNION_DATA_Z_LSB_ADDR 		= 0X26        #同様
	BNO055_QUATERNION_DATA_Z_MSB_ADDR 		= 0X27        #同様

	# Linear acceleration data registers (線形加速度データ定義)
	BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 		= 0X28        #同様
	BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR 		= 0X29        #同様
	BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR	 	= 0X2A        #同様
	BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR		= 0X2B        #同様
	BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR		= 0X2C        #同様
	BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR		= 0X2D        #同様

	# Gravity data registers (重力加速度データ定義)
	BNO055_GRAVITY_DATA_X_LSB_ADDR 			= 0X2E        #同様
	BNO055_GRAVITY_DATA_X_MSB_ADDR	 		= 0X2F        #同様
	BNO055_GRAVITY_DATA_Y_LSB_ADDR 			= 0X30        #同様
	BNO055_GRAVITY_DATA_Y_MSB_ADDR 			= 0X31        #同様
	BNO055_GRAVITY_DATA_Z_LSB_ADDR 			= 0X32        #同様
	BNO055_GRAVITY_DATA_Z_MSB_ADDR 			= 0X33        #同様

	# Temperature data register (温度データ定義)
	BNO055_TEMP_ADDR 				= 0X34        #同様

	# Status registers 
	BNO055_CALIB_STAT_ADDR 				= 0X35        #このバイトは2bitの塊ごとに計四つのステータスが格納されている。0x35;(システムcalib状態, ジャイロcalib状態, 加速度calib状態, 地磁気calib状態)　00;未キャリブレーション, 11;完全キャリブレーション
	BNO055_SELFTEST_RESULT_ADDR	 		= 0X36        #self-test用の関数。機器を呼び出すイメージ。このbyteの構成は(予約済み4bit, マイコン1bit, ジャイロ1bit, 地磁気1bit , 加速度1bit)であり、0;失敗、1;成功
	BNO055_INTR_STAT_ADDR 				= 0X37        #interrupt（割り込み）検知　構成は(加速度no motion 1bit, 加速度any motion 1bit, 加速度high-g 1bit, ジャイロdata ready 1bit, ジャイロhigh rate 1bit, ジャイロany motion 1bit, 地磁気data ready 1bit, 加速度/bsx 1bit)　(1(7),)で長時間動きがない

	BNO055_SYS_CLK_STAT_ADDR 			= 0X38        #クロック（時計）が設定されているか　構成は(予約済み7bit, 時計状態1bit) 0;設定可能　1;設定不可(内部or外部機器により設定済み)
	BNO055_SYS_STAT_ADDR 				= 0X39        #p74　システムを表す　構成は(定義なし1bit, センサフュージョンなしで動作中1bit, センサーフュージョンアルゴリズムが動作中1bit, self-test実行中1bit, システム全体初期化中1bit, 周辺機器を初期化中1bit, システムエラー発生中1bit, システムアイドリングストップ1bit)　0;問題なし　1;実行中or問題あり
	BNO055_SYS_ERR_ADDR 				= 0X3A        #p75　システムエラーを表す　1byteすべて使用　0;no error, 1;周辺機器の初期化エラー, 2;システム初期化エラー, 3;self-test失敗, 4;レジスタ値が範囲外, 5;無効なレジスタアドレスにアクセス, 6;レジスタへの書き込み失敗, 7;セルフテスト設定が正しくない, 8無効または不適切なモード設定, 9;センサ設定のエラー, A;電源モードとセンサ設定が矛盾している
	# Unit selection register 
	BNO055_UNIT_SEL_ADDR 				= 0X3B        #p75　(動作モード1bit(0;windows, 1;android), 予約済み2bit, 温度の単位1bit(0;セルシウス温度℃, 1;ファーレンヘイト℉), オイラー角の単位1bit(0;度, 1;rad), 角速度の単位1bit(0;dps. 1;rps), 加速度の単位1bit(0;m/s^2, 1;mil_gravity))
	BNO055_DATA_SELECT_ADDR 			= 0X3C        #

	# Mode registers 
	BNO055_OPR_MODE_ADDR 				= 0X3D        #p76　動作モードレジスタ　構成は(予約済み4bit, オペレーションモード4bit)　多分使わないので自分で見ましょう(使いました)
	BNO055_PWR_MODE_ADDR 				= 0X3E        #p76　パワーモード設定　構成は(予約済み6bit, パワーモード2bit)　00;normal, 01;low power, 10;suspend, 11;invalid

	BNO055_SYS_TRIGGER_ADDR 			= 0X3F        #p76　システムトリガ制御　構成は(発信機1bit(0;内部発振器の使用, 1;外部発振器の使用), 割り込み操作1bit(1;すべての割り込み操作を中断), システムリセット1bit(1;システムリセット), 予約済み4bit, セルフテスト1bit(1;セルフテストを行う) )
	BNO055_TEMP_SOURCE_ADDR 			= 0X40        #p76 データシートに誤り在り　おそらく構成は(予約済み6bit, 温度制御2bit) 00;加速度センサ内部の温度センサを利用, 01;ジャイロスコープ内部の温度センサを利用

	# Axis remap registers 
	BNO055_AXIS_MAP_CONFIG_ADDR 			= 0X41        #p77 座標マッピング 構成は(予約済み2bit, z軸マッピング2bit, y軸マッピング2bit, x軸マッピング2bit) 00;x軸, 01;y軸, 10;z軸, 11;invalid
	BNO055_AXIS_MAP_SIGN_ADDR 			= 0X42        #p77 軸の反転設定 構成は(予約済み5bit, x-axis 1bit, y-axis 1bit, z-axis 1bit) 0;positive正, 1;negative負

	# SIC registers 
	BNO055_SIC_MATRIX_0_LSB_ADDR 			= 0X43        #p77 ソフトアイアン行列の最初のバイトを示す。ソフトアイアン行列を利用すると、センサが補正した磁場データを補正できる。
	BNO055_SIC_MATRIX_0_MSB_ADDR 			= 0X44        #p77 0要素上位8bit
	BNO055_SIC_MATRIX_1_LSB_ADDR 			= 0X45        #p77 1要素下位8bit
	BNO055_SIC_MATRIX_1_MSB_ADDR 			= 0X46        #p77 1要素上位8bit
	BNO055_SIC_MATRIX_2_LSB_ADDR 			= 0X47        #同様
	BNO055_SIC_MATRIX_2_MSB_ADDR 			= 0X48        #同様
	BNO055_SIC_MATRIX_3_LSB_ADDR 			= 0X49        #同様
	BNO055_SIC_MATRIX_3_MSB_ADDR 			= 0X4A        #同様
	BNO055_SIC_MATRIX_4_LSB_ADDR 			= 0X4B        #同様
	BNO055_SIC_MATRIX_4_MSB_ADDR 			= 0X4C        #同様
	BNO055_SIC_MATRIX_5_LSB_ADDR 			= 0X4D        #同様
	BNO055_SIC_MATRIX_5_MSB_ADDR 			= 0X4E        #同様
	BNO055_SIC_MATRIX_6_LSB_ADDR 			= 0X4F        #同様
	BNO055_SIC_MATRIX_6_MSB_ADDR 			= 0X50        #同様
	BNO055_SIC_MATRIX_7_LSB_ADDR 			= 0X51        #同様
	BNO055_SIC_MATRIX_7_MSB_ADDR 			= 0X52        #同様
	BNO055_SIC_MATRIX_8_LSB_ADDR 			= 0X53        #同様
	BNO055_SIC_MATRIX_8_MSB_ADDR 			= 0X54        #同様
	
	# Accelerometer Offset registers	 
	ACCEL_OFFSET_X_LSB_ADDR 			= 0X55        #p81 加速度補正値 x-axis 下位8bit
	ACCEL_OFFSET_X_MSB_ADDR 			= 0X56        #p82 加速度補正値 x-axis 上位8bit
	ACCEL_OFFSET_Y_LSB_ADDR 			= 0X57        #p82 加速度補正値 y-axis 下位8bit
	ACCEL_OFFSET_Y_MSB_ADDR 			= 0X58        #p82 加速度補正値 y-axis 上位8bit
	ACCEL_OFFSET_Z_LSB_ADDR 			= 0X59        #p82 加速度補正値 z-axis 下位8bit
	ACCEL_OFFSET_Z_MSB_ADDR 			= 0X5A        #p83 加速度補正値 z-axis 上位8bit

	# Magnetometer Offset registers 
	MAG_OFFSET_X_LSB_ADDR 				= 0X5B        #p83 地磁気補正値 x-axis 下位8bit
	MAG_OFFSET_X_MSB_ADDR 				= 0X5C        #p83 地磁気補正値 x-axis 上位8bit
	MAG_OFFSET_Y_LSB_ADDR 				= 0X5D        #p83 地磁気補正値 y-axis 下位8bit
	MAG_OFFSET_Y_MSB_ADDR 				= 0X5E        #p84 地磁気補正値 y-axis 上位8bit
	MAG_OFFSET_Z_LSB_ADDR 				= 0X5F        #p84 地磁気補正値 z-axis 下位8bit
	MAG_OFFSET_Z_MSB_ADDR 				= 0X60        #p84 地磁気補正値 z-axis 上位8bit

	# Gyroscope Offset registers
	GYRO_OFFSET_X_LSB_ADDR 				= 0X61        #p84 ジャイロ補正値 x-axis 下位8bit
	GYRO_OFFSET_X_MSB_ADDR 				= 0X62        #p85 ジャイロ補正値 x-axis 上位8bit
	GYRO_OFFSET_Y_LSB_ADDR 				= 0X63        #p85 ジャイロ補正値 y-axis 下位8bit
	GYRO_OFFSET_Y_MSB_ADDR 				= 0X64        #p85 ジャイロ補正値 y-axis 上位8bit
	GYRO_OFFSET_Z_LSB_ADDR 				= 0X65        #p85 ジャイロ補正値 z-axis 下位8bit
	GYRO_OFFSET_Z_MSB_ADDR 				= 0X66        #p86 ジャイロ補正値 z-axis 上位8bit

	# Radius registers 
	ACCEL_RADIUS_LSB_ADDR 				= 0X67        #p86 加速度全体補正値 下位8bit
	ACCEL_RADIUS_MSB_ADDR 				= 0X68        #p86 加速度全体補正値 上位8bit
	MAG_RADIUS_LSB_ADDR 				= 0X69        #p86 地磁気全体補正値 下位8bit
	MAG_RADIUS_MSB_ADDR 				= 0X6A        #p86 地磁気全体補正値 上位8bit

	# REGISTER DEFINITION END(レジスタの割り当てが終わりましたよということです)

        #init関数;パケージの初期化に使われる (self, ・・・)とするのは関数外の変数にアクセスするため(今回の場合はOPERATION_MODE_NDOF,　つまりselfはクラス内の関数を用いるときに必要（BNO055に固有） )
	def __init__(self, sensorId=-1, address=0x28):                #0x28アドレスはi2c通信アドレス
		self._sensorId = sensorId                             #渡されたセンサidをsensorIdに格納
		self._address = address                               #I2Cのデバイスアドレスを_adressに保存する
		self._mode = BNO055.OPERATION_MODE_NDOF               #センサをバランスよく使えるNDOFモードを_modeに保存する(関数を持ってくるときはモジュール名.関数名であるのでBNO055は必要)

	#bigin関数
	def begin(self, mode=None):                                   #mode値が設定されていなければ
		if mode is None: mode = BNO055.OPERATION_MODE_NDOF    #NDOF(地磁気、ジャイロ、加速度センサ＋地磁気校正あり)に設定
		# Open I2C bus(smbusはI2C通信を行うためのpythonライブラリ)
		self._bus = smbus.SMBus(1)                            #I2Cパス1を開放することを_busとして格納 ラズパイ上のI2C1のこと

		# Make sure we have the right device
		if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:         # 危機ID=0なら
			time.sleep(1)	# Wait for the device to boot up                      #スリープ時間 1 s
			if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID: # 危機ID=0なら
				return False                                                  #失敗！！

		# Switch to config mode
		self.setMode(BNO055.OPERATION_MODE_CONFIG)                                    #selfとはpythonのクラスの中で「そのインスタンス自身」を表す特別な引数 受け渡しのみ行われる

		# Trigger a reset and wait for the device to boot up again
		self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x20])                       #125行目参照 writeBytes(A, B)はAにBを書き込む 外部発振器の使用とシステム(ソフトウェア)リセットを行う
		time.sleep(1)                                                                 #1 s待機
		while self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:      #readBytes(A)でAを読み込む [0]のbit読み込みデータシート参照 起動すればBN055idは0xA0となる→ループ脱出
			time.sleep(0.01)                                                      #0.01 s待機
		time.sleep(0.05)                                                              #0.05 s待機

		# Set to normal power mode
		self.writeBytes(BNO055.BNO055_PWR_MODE_ADDR, [BNO055.POWER_MODE_NORMAL])      #125行参照 Power Modeを標準状態にする
		time.sleep(0.01)                                                              #0.01 s待機

		self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])                              #39行目参照 ページ0に変更 念のため戻すらしい→意味わからん
		self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0])                          #127行目参照 先ほどのソフトウェアリセットを解除
		time.sleep(0.01)                                                              #0.01 s待機

		# Set the requested mode
		self.setMode(mode)                                                            #194行目参照 OPERATION_MODE_NDOFに設定する
		time.sleep(0.02)                                                              #0.02 s待機

		return True                                                                   #trueを返す

	#setMode関数の定義
	def setMode(self, mode):                                                              
		self._mode = mode                                                             #mode値を_modeに格納
		self.writeBytes(BNO055.BNO055_OPR_MODE_ADDR, [self._mode])                    #_modeを動作モードレジスタに移す
		time.sleep(0.03)                                                              #0.03 s待機 モード変更の際には必須　ないとバグっちゃう

	#外部クリスタルを使うか(水晶振動子の外付けコンデンサ)
	def setExternalCrystalUse(self, useExternalCrystal = True):                                      
		prevMode = self._mode                                                                    #self._modeをprevModeに格納
		self.setMode(BNO055.OPERATION_MODE_CONFIG)                                               #16行目参照　configモードに入る
		time.sleep(0.025)                                                                        #0.025秒待機
		self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])                                         #39行目参照　ページidレジスタに0を書き込む⇒データ取得のレジスタ群ページに戻る
		self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x80] if useExternalCrystal else [0])   #
		time.sleep(0.01)                                                                         #
		self.setMode(prevMode)
		time.sleep(0.02)

	#システムステータスを参照する　行列形式
	def getSystemStatus(self):
		self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
		(sys_stat, sys_err) = self.readBytes(BNO055.BNO055_SYS_STAT_ADDR, 2)
		self_test = self.readBytes(BNO055.BNO055_SELFTEST_RESULT_ADDR)[0]
		return (sys_stat, self_test, sys_err)

	#センサのverを参照する　行列形式
	def getRevInfo(self):
		(accel_rev, mag_rev, gyro_rev) = self.readBytes(BNO055.BNO055_ACCEL_REV_ID_ADDR, 3)
		sw_rev = self.readBytes(BNO055.BNO055_SW_REV_ID_LSB_ADDR, 2)
		sw_rev = sw_rev[0] | sw_rev[1] << 8
		bl_rev = self.readBytes(BNO055.BNO055_BL_REV_ID_ADDR)[0]
		return (accel_rev, mag_rev, gyro_rev, sw_rev, bl_rev)

	#各センサのキャリブレーション状態を参照する　戻り値:(加速度, 磁気, ジャイロ, システム)
	def getCalibration(self):
		calData = self.readBytes(BNO055.BNO055_CALIB_STAT_ADDR)[0]
		return (calData >> 6 & 0x03, calData >> 4 & 0x03, calData >> 2 & 0x03, calData & 0x03)

	#内臓温度センサ値を取得する　戻り値:整数値
	def getTemp(self):
		return self.readBytes(BNO055.BNO055_TEMP_ADDR)[0]

	#指定タイプ(地磁気, ジャイロ, 姿勢(オイラー角:heading, roll, pitch), 重力, 線形加速度,  etc)の取得　戻り値：(x, y, z)
	def getVector(self, vectorType):
		buf = self.readBytes(vectorType, 6)
		xyz = struct.unpack('hhh', struct.pack('BBBBBB', buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]))
		if vectorType == BNO055.VECTOR_MAGNETOMETER:	scalingFactor = 16.0
		elif vectorType == BNO055.VECTOR_GYROSCOPE:	scalingFactor = 900.0
		elif vectorType == BNO055.VECTOR_EULER: 		scalingFactor = 16.0
		elif vectorType == BNO055.VECTOR_GRAVITY:	scalingFactor = 100.0
		else:											scalingFactor = 1.0
		return tuple([i/scalingFactor for i in xyz])

	def getQuat(self):
		buf = self.readBytes(BNO055.BNO055_QUATERNION_DATA_W_LSB_ADDR, 8)
		wxyz = struct.unpack('hhhh', struct.pack('BBBBBBBB', buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]))
		return tuple([i * (1.0 / (1 << 14)) for i in wxyz])

	def readBytes(self, register, numBytes=1):
		return self._bus.read_i2c_block_data(self._address, register, numBytes)

	def writeBytes(self, register, byteVals):
		return self._bus.write_i2c_block_data(self._address, register, byteVals)
	def get_heading(self):
		heading = self.getVector(self.VECTOR_EULER)[0]
		return heading

if __name__ == '__main__':
    bno = BNO055()
    if bno.begin() is not True:
        print("Error initializing device")
        exit()
    time.sleep(1)
    bno.setMode(BNO055.OPERATION_MODE_NDOF)
    # ==== キャリブレーション完了待機 ====
    #(トリプルくうぉーテーションでコメントアウト)
    """
    print("キャリブレーション中... センサをいろんな向きにゆっくり回してください")
    while True:
        sys, gyro, accel, mag = bno.getCalibration()
        print(f"Calib → Sys:{sys}, Gyro:{gyro}, Acc:{accel}, Mag:{mag}", end='\r')
        if gyro == 3 and mag == 3:
            print("\n✅ キャリブレーション完了！")
            break
        time.sleep(0.5)
    """
    time.sleep(5)
    bno.setExternalCrystalUse(True)

    while True:
        for i in range(20):
            print(bno.getVector(BNO055.VECTOR_EULER))
            a = bno.getVector(BNO055.VECTOR_EULER)
            B = a[0]
            print(B)
            c = bno.get_heading()
            print(c)
            time.sleep(0.1)

