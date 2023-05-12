#ifndef ARMOR_DETECTOR_HPP
#define ARMOR_DETECTOR_HPP
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include<iostream>

namespace armor_detector
{

//闂備浇妗ㄧ粈浣圭附閺傚晝褎寰勭€ｅ墎鐭楀┑鐘诧工濡藟閿燂拷
enum EnemyColor
{
	ALL = 0,
	BLUE = 1,
	RED = 2
};

struct ImageConfigs
{

};

//闂備浇顕уù姘跺垂閸楃儐鐒芥い鏍仜閻鏌涚仦鍓р姇婵炲牞鎷�
struct LightConfigs
{
	//闂備浇顕уù姘跺垂閸楃儐鐒芥い鏍仜瀹告繈鏌曟繛鐐珕婵綇鎷�
	int perimeter_max = 1000;
	int perimeter_min = 16;
	//
	float maxLightArea = 0;
	float minLightArea = 0;
	//闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵€岄柛銊ョ埣瀵鏁愭径濠勵吅闂佹寧绻傞幉娑㈠箻缂佹鍘搁梺鍛婁緱閸犳宕愰幇鐗堢厸鐎光偓鐎ｎ剛鐦堥悗瑙勬礃鐢帟鐏掗柣鐐寸▓閳ь剙鍘栨竟鏇㈡⒑閸濆嫮鈻夐柛瀣у亾闂佺ǹ顑嗛幐鎼侊綖濠靛鏁嗛柛灞剧敖閵娾晜鈷戦柛婵嗗椤箓鏌涢弮鈧崹鍧楃嵁閸愵喖顫呴柕鍫濇噹缁愭稒绻濋悽闈浶㈤悗姘间簽濡叉劙寮撮姀鈾€鎷绘繛杈剧到閹芥粎绮旈悜妯镐簻闁靛闄勫畷宀€鈧娲橀〃鍛达綖濠婂牆鐒垫い鎺嗗亾妞ゆ洩缍侀、鏇㈡晝閳ь剛绮婚懡銈囩＝濞达絽顫栭鍛洸濠靛倸鎲￠埛鎴︽煟閹邦垱顥夐柛鏃€宀搁弻娑㈡偄闁垮浠撮梺璇″灠閸熸挳寮幘缁樺亹鐎规洖娲ら獮宥呪攽閿涘嫬浠滄い鎴濇噺椤ㄣ儵骞栨担绋垮亶闂佸湱铏庨崰妤呮偂濞戞◤褰掓晲閸偅缍堥悗瑙勬礀閻ジ鍩€椤掍緡鍟忛柛锝庡櫍瀹曟粓鎮㈤梹鎰畾闂佸壊鍋呭ú鏍嵁閵忊€茬箚闁靛牆鎷戝妤冪磼閹插瀚�
	int light_draw = 0;
	int light_edit = 0;

	//闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵€岄柛銊ョ埣瀵鏁愭径濠勵吅闂佹寧绻傞幉娑㈠箻缂佹鍘搁梺鍛婁緱閸樹粙骞夐崫銉х＜閺夊牄鍔屽ù顔锯偓娈垮櫘閸ｏ綁鐛€ｎ亖鏀介柛鈩冪懐閸熷牓姊婚崒娆愵樂缂侀硸鍠氶埀顒勬涧閻倸鐣烽幎鑺ユ櫜濠㈣泛锕ラ悗顒勬⒑瑜版帒浜伴柛銊﹀▕閹潡鍩€椤掑嫭鈷戦柛婵嗗閳ь剙缍婇、鏍р枎閹邦剦鍤ら梺纭呭吹缁岋箠ght/width
	float maxAspectRatio = 30;
	float minAspectRatio = 1;

    float maxAngle = 45;
	float minAngle= 45;
};

//闂佽崵鍠嶇粈渚€骞婇幇鏉挎瀬闁规儼妫勭痪褔鏌涢…鎴濇灈婵☆偅锕㈤弻鈩冪瑹婵犲啫顏�
struct ArmorConfigs
{
	//
	int armor_draw = 1;
	int armor_edit = 0;
	// 
	int light_height_ratio_min = 5;
	int light_height_ratio_max = 20;
	// 
	int light_width_ratio_min = 5;
	int light_width_ratio_max = 30;
	// 
	int light_y_different = 10;
	// 
	int light_height_different = 10;
	// 
	int armor_angle_different = 80;
	// 
	int small_armor_aspect_min = 11;
	int armor_type_th = 33;
	int big_armor_aspect_max = 42;
};


//struct LightData
//{
//	float width=0;
//	float height=0;
//	float area=0;
//
//	
//	cv::RotatedRect lightRect;
//};

struct ArmorData
{
	float width = 0.f;
	float height = 0.f;
	float area = 0.f;
	float tan_angle = 0.f;
	float aspectRatio = 0.f;
	float distance_center = 0.f;
	
	float left_light_width = 0.f;
	float right_light_width = 0.f;
	float left_light_height = 0.f;
	float right_light_height = 0.f;
	float light_height_aspect = 0.f;
	float light_width_aspect = 0.f;
	
	cv::Point2f armor_lt;
	cv::Point2f armor_rb;
	cv::Point2f p0;
	cv::Point2f p1;
	cv::Point2f p2;
	cv::Point2f p3;
	cv::Point2f armor_points[4];

	int distinguish = 0;      //  0  1 
	//int distance_center = 0;      // 
	cv::RotatedRect armorRect;
	cv::RotatedRect leftLight;
	cv::RotatedRect rightLight;
};

class ArmorDetector
{
public:
	
	/**
	* @brief 闂備礁鎼鍛偓姘嵆閸┾偓妞ゆ帒鍊稿瓭闂佹悶鍊濇禍璺侯嚕椤愶附鏅搁柨鐕傛嫹
	* @param 
	*/
	ArmorDetector();
	
	/**
	* @brief 闂備礁鎼鍡涘礉閺嶎厽鍊垫い鏍仜缁€鍕煠閹帒鍔滄繛鍫嫹
	* @param
	*/
	~ArmorDetector();
	
	/**
	* @brief 闂傚倷鐒﹁ぐ鍐晝閵夆晛妫樺〒姘ｅ亾鐎规洘锕㈠畷銊╊敇閻樿櫕鍠�
	*/
	void release();
	
	/**
	* @brief 闂備浇顕栭崜婵嬵敋瑜斿畷锝夊幢濞戞鐓戦梺鎸庣箓閹冲骸袙閹扮増鐓涘〒姘攻鐎氾拷
	* @param 闂備礁鎲￠…鍥窗鎼粹埗褰掓晸閿燂拷 闂備浇妗ㄧ粈渚€寮甸鍕摕闁绘棃娼荤槐锝嗙節婵犲倹顥滄い銉嫹
	*/
	void runArmorDetector(const cv::Mat& srcImg, const EnemyColor enemy_color);
	
	/**
	* @brief	闂佹悶鍎查崕鎶藉磿濮橆叏绱ｉ柛鏇ㄥ亽濡查亶鏌ｉ悜鍥ㄥ
	* @param	闂佸憡顭囬崰搴∶归敓锟� 闂佽桨绀侀張顒勫蓟閻旈潻绱ｆ繝濠傛椤ワ拷
	* @return	
	*/
	cv::Mat imgPretreat(const cv::Mat & srcImg,const EnemyColor enemy_color);
	
	/**
	* @brief 闂佸搫瀚烽崹浼村箚娓氣偓楠炲秹骞嗚閻撳倿鏌ｈ箛锝呯仭婵炵》鎷�
	* @param 
	* @return
	*/
	bool isFindLights();

	/**
	* @brief 
	* @param
	* @return
	*/
	int findArmor();

	/**
	* @brief 
	* @param 
	* @return
	*/
	bool matchLights(const int i, const int j);
	
	/**
	* @brief 
	* @param 
	* @return
	*/
	void chooseTarget();

	/**
	* @brief 闂佽崵濮崇欢銈囨閺囥垺鍋╃紓浣诡焽閳绘棃鏌曢崼婵囨悙濞寸媭鍨跺濠氬磼濠婂孩顥撶紓浣瑰姈缁嬫挾鍒掔€ｎ喗鏅搁柨鐕傛嫹
	* @param 
	* @return
	*/
	float getDistance(const cv::Point a, const cv::Point b);

	void getAnglediff();

	int getArmorType();

	

	std::vector<cv::Point2f> returnFinalArmorRect();
private:
	cv::Mat image;
	cv::Mat _grayImg;
	cv::Mat split_image;
	cv::Mat gray_image;
	cv::Mat _binImg;


	LightConfigs _lightConfigs;
	ArmorConfigs _armorConfigs;
	//ArmorData _armorData;

	std::vector<cv::Mat> channels;
	std::vector<cv::Point2f> image_points;
	//LightData lightData;
	ArmorData armorData;
	
	std::vector<cv::RotatedRect> light;
	
	std::vector<cv::RotatedRect> armor;
	//std::vector<ArmorData> armor;
	/*std::vector<LightData> light;
	std::vector<ArmorData> armor;*/

	
};

}
#endif