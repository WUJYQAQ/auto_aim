#ifndef POSE_HPP
#define POSE_HPP
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include<iostream>


enum ArmorType
{
	smallArmor = 0,
	bigArmor = 1
};

struct PnP_Results
{
	float yaw_angle;
	float pitch_angle;
	int distance;
	PnP_Results() 
	{
    yaw_angle   = 0.f;
    pitch_angle = 0.f;
    distance       = 0;
  	}
};

struct PnPConfig
{
	int smallArmorHeight = 60;
	int smallArmorWidth = 140;

	int bigArmorHeight = 60;
	int bigArmorWidth = 245;
};



class PoseSolver
{
public:


	//CalculateResults calResults;

	//CalculateResults* calResult = (CalculateResults*)malloc(sizeof(calResults));//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝棙鍎柣鐔哥懃鐎氼剟鎯冩ィ鍐┾拻濞达絽鎲￠幆鍫熴亜閹存繃鍣介柍褜鍓氶惌顕€宕￠崘宸殨妞ゆ劧绠戠粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫濋唶闁哄洨鍠撻崢鐢电磽娴ｅ壊鍎愰悗绗涘洤绐楁慨妞诲亾闁哄备鍓濋幏鍛喆閸曨偆锛撻柣搴㈩問閸ｎ噣宕滈悢闀愮箚闁割偅娲栭獮銏′繆閵堝拑姊楃紒鎲嬬畵濮婂宕掑▎鎰偘濠电偛顦板ú鐔风暦娴兼潙绠涢柡澶婄仢閸ゆ垶绻濋悽闈浶ｉ柤鐟板⒔缁顢涢悙瀵稿幗闂佺鎻徊鍊燁暱闂備礁鎼幊蹇涘箖閸岀偛钃熼柨鐔哄Т绾惧吋淇婇婵勨偓鈧柛瀣崌瀵粙顢曢悢铚傚濠殿喗锕╅崜娑氱矓濞差亝鐓涢悘鐐额嚙婵倿鏌熼鍝勭伈鐎规洏鍔嶇换婵嬪礋椤忓棛锛熼梻鍌氬€风粈浣虹礊婵犲泚澶愬箻鐠囪尙顦у┑顔姐仜閸嬫挾鈧鍣崑濠囩嵁濡皷鍋撳☉娅亪锝為垾鏂ユ斀闁绘劕寮堕ˉ婊呮喐閹殿喖浠辨鐐插暙椤撳吋寰勭€ｎ剙骞嶅┑锛勫仜椤戝懏顨ラ幖浣稿偍闁圭虎鍠楅悡娑樏归敐鍫綈鐎规洖鐬奸埀顒侇問閸ｎ噣宕滈悢闀愮箚闁割偅娲栭獮銏′繆閵堝拑姊楃紒鎲嬬畵濮婂宕掑▎鎰偘濠电偛顦板ú鐔风暦娴兼潙绠涢柡澶婄仢閸ゆ垶绻濋悽闈浶ｉ柤鐟板⒔缁顢涢悙瀵稿幗闂佺鎻徊鍊燁暱闂備礁鎼幊蹇涘箖閸岀偛钃熼柕鍫濐槸娴肩娀鏌涢弴銊ヤ簮闁稿鎸荤粋鎺斺偓锝庝簽椤斿苯顪冮妶鍛闁绘妫涚划濠氼敍濠ф儳浜鹃柣鐔告緲椤忣亝绻濋姀鈽嗙劷缂侇噯绲借灃闁告侗鍠掗幏娲煟閻樺厖鑸柛鏂胯嫰閳诲秹骞囬悧鍫㈠幍闂佸憡鍨崐鏍偓姘炬嫹

	
	
	PoseSolver()=default;
	explicit PoseSolver(const char* filePath, int camId);

	~PoseSolver(); 

	//CalculateResults* poseSolve();	//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻濠氭倵濞戞顏勵嚕閸ф鈷戠憸鐗堝笒娴滀即鏌涘Ο鍦煓闁诡喒鈧枼鏋庨柟鍓цˉ閹锋椽鏌ｉ悩鍏呰埅闁告柨鐭傚鎼佸箣閿旂晫鍘搁柣搴秵娴滃爼鎮￠崗纰辨闁绘劘灏欑粻濠氭煛娴ｈ宕岄柡浣规崌閺佹捇鏁撻敓锟�


	void setCameraParams(const cv::Mat& camMatrix, const cv::Mat& distCoeffs);//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭骞栧ǎ顒€濡奸柣鎺戠仛閵囧嫰骞掗幋婵冨亾閹间礁鍑犻幖娣妽閻撶喖鏌ㄥ┑鍡╂Ш闁伙絾妞介弻锛勪沪鐠囨彃顫囬梺璇″枔閸ㄨ棄鐣烽妸锔剧瘈闁告洦浜炵壕鍧楁⒒閸屾瑧顦︾紓宥咃躬钘熼柟鎹愵嚙缁€鍌涙叏濡炶浜鹃悗娈垮櫘閸嬪﹪鐛Ο鍏煎珰闁告瑥顦藉Λ鐔兼⒒娓氣偓濞佳囨偋閸℃あ娑樜旈崪浣规櫆闂佸壊鍋呭ú姗€鍩涢幋锔藉仯闁诡厽甯掓俊濂告煛鐎ｎ偄鐏撮柡宀嬬磿閳ь剨缍嗘禍鍫曟偂閸忕⒈娈介柣鎰皺缁犲鏌熼鐣岀煉闁瑰磭鍋ゆ俊鐑芥晜缁涘鎴烽梻鍌氬€峰ù鍥р枖閺囥垹闂柨鏇炲€哥粻顖炴煥閻曞倹瀚�

	int readFile(const char* filePath, int camId);//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ磭妲愰幒鎴犳殕濠电姴鍟鐒為梻鍌氬€搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鈧箍鍎遍ˇ浼村吹閹达箑绠归弶鍫濆⒔缁嬪鏌涜箛鏃傜煉婵﹥妞藉畷褰掝敋閸涱厼澹堟俊鐐€曠换鎰版偉閻撳寒娼栭柧蹇氼潐瀹曞鏌曟繛鍨姕闁诲酣绠栧娲传閸曢潧鍓板銈庡幘閸忔﹢鎮伴璺ㄧ杸闁规崘鍩栧▓婵嬫⒑閸濆嫷妲兼繛澶嬫礋楠炲啯绺介崨濞炬嫽婵炶揪缍€婵倗娑甸崼鏇熺厱闁绘ê鍟挎慨宥団偓娈垮枛閹诧紕鎹㈠┑鍡╂僵妞ゆ帒鍋嗛崬鐢告⒒娴ｈ櫣甯涢柛銊ュ悑閹便劑濡舵径濠勬煣闂佸綊妫块悞锕傛偂濞戙垺鐓曢悘鐐扮畽椤忓牆鐒垫い鎺嶈兌婢ч亶鏌嶈閸撴岸鎳濋崜褏绀婂┑鐘叉搐閽冪喖鏌曟繛鐐珕闁稿瀚伴弻娑樷槈濮楀牆濮涙繛瀵稿帶鐎氭澘顫忔ウ瑁や汗闁圭儤绻冮ˉ鏍⒑缁嬭法绠查柨鏇樺灩椤曪綁顢曢敃鈧粻鑽ょ磽娴ｈ偂鎴濃枍閵忋倖鈷戦悹鎭掑妼濞呮劙鏌熼崙銈嗗

	void setObjPoints(ArmorType type, double width, double height);//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆娴ｇ晫顢呴梻浣烘嚀瀵爼銆冩繝鍌滄殾闁靛ě鈧崑鎾斥槈濞呰櫕鍨甸埢宥夊閵堝棌鎷婚梺绋挎湰閼归箖寮抽姀锛勭濠㈣泛顑囬埊鏇㈡煙椤栨稒鐓ユ顏冨嵆瀹曠厧鈹戦崼顐㈡暩闂傚倷鑳剁划顖炲垂閸忓吋鍙忛柕鍫濐槸閻ら箖鏌熼梻瀵稿妽闁绘挻鐩弻娑氫沪娑斿拋浜崺鈧い鎺嶈兌婢ч亶鏌嶈閸撴岸鎳濋崜褏绀婂┑鐘叉搐閽冪喖鏌曟繛鐐珕闁稿瀚伴弻娑樷槈濮楀牆濮涙繛瀵稿帶鐎氭澘顫忔ウ瑁や汗闁圭儤绻冮ˉ鏍⒑缁嬭法绠查柨鏇樺灩椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫濋唶闁哄洨鍟块幏娲煟閻樺厖鑸柛鏂跨焸瀵悂骞嬮敂鐣屽幐闁诲函缍嗘禍鍫曟偂閸忕⒈娈介柣鎰皺缁犲鏌℃担瑙勫磳闁轰焦鎹囬弫鎾绘晸閿燂拷

	void getImgpPoints(std::vector<cv::Point2f> image_points);//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰叏婵犲偆鐓肩€规洘锕㈤、姗€鎮㈤幁鎺嗗亾閹扮増鐓欓柣鎾虫捣閹界姷绱掗鑺ュ磳闁绘侗鍠涚粻娑樷槈濞嗗繋鐥俊鐐€栭悧顓犲緤妤ｅ叝澶嬬節閸ャ劉鎷绘繛杈剧秬婵倗娑甸崼鏇熺厱闁绘ê鍟挎慨宥団偓娈垮枛閹诧紕鎹㈠┑鍡╂僵妞ゆ帒鍋嗛崬鐢告⒒娴ｈ櫣甯涢柛銊ュ悑閹便劑濡舵径濠勬煣闂佸綊妫块悞锕傛偂濞戙垺鐓曢悘鐐扮畽椤忓牆鐒垫い鎺嶈兌婢ч亶鏌嶈閸撴岸鎳濋崜褏绀婂┑鐘叉搐閽冪喖鏌曟繛鐐珕闁稿瀚伴弻娑樷槈濮楀牆濮涙繛瀵稿帶鐎氭澘顫忔ウ瑁や汗闁圭儤绻冮ˉ鏍⒑缁嬭法绠查柨鏇樺灩椤曪綁顢曢敃鈧粻濠氭偣閸パ冪骇妞ゃ儲绻堝娲濞戞艾顣哄┑鈽嗗亝椤ㄥ﹪銆侀弮鍫熸櫢闁跨噦鎷�

	void solvePose(int armorType);

	float getYawAngle();

	float getPitchAngle();

	int getDistance();

	std::tuple<double,double,double> getPose();

	void runPoseSolver();
	
private:

	PnP_Results pnp_results;

	cv::Mat instantMatrix;	//Camera Matrix
	cv::Mat distortionCoeffs;	//Distortion Coeffs of Camera

	std::vector<cv::Point3f> bigObjPoints;
	std::vector<cv::Point3f> smallObjPoints;

	std::vector<cv::Point2f> imagePoints;

	//cv::Mat rvec ;
	//cv::Mat tvec ;

	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);	//
		//
	
	double pitch, yaw, distance;
	
	ArmorType armorType;

	double GUN_CAM_DISTANCE_Y;

};


#endif 