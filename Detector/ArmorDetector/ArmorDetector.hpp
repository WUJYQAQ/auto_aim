#ifndef ARMOR_DETECTOR_HPP
#define ARMOR_DETECTOR_HPP
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include<iostream>

namespace armor_detector
{

//闂佽桨绀佹總鏂啃ф径瀣剁矗婵犲﹤妫楅ˉ锟�
enum EnemyColor
{
	ALL = 0,
	BLUE = 1,
	RED = 2
};

struct ImageConfigs
{

};

//闂佽娴氶崹鍗烆焽椤栫偛鐭楅柛灞剧⊕濞堬拷
struct LightConfigs
{
	//闂佽娴氶崹鍗烆焽椤栫偛宸濋柕濞炬櫆濮ｏ拷
	int perimeter_max = 1000;
	int perimeter_min = 16;
	//
	float maxLightArea = 0;
	float minLightArea = 0;
	//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍠楅崐鎰版煛瀹€瀣瘈鐎规洘甯掕灒閻炴稈鈧厖澹曢梺鍝勭▉閸嬧偓闁稿鎸搁～婵嬵敆閸屾簽銊╂⒑閸濆嫯顫﹂柛鏃€鍨块獮鍐閵堝懎绐涙繝鐢靛Т鐎氼亞妲愰弴銏♀拻濞达絽鎽滅粔鐑樸亜閵夛附宕岀€规洘顨呴～婊堝焵椤掆偓椤曪綁顢曢敃鈧粻鑽ょ磽娴ｅ顏呯婵傚憡鈷戦柣鎰閸旀岸鏌涢悢閿嬪仴闁诡垰鍟撮弫鎾绘偐瀹曞洤骞嶅┑锛勫仜椤戝懏顨ラ幖浣稿偍闁圭虎鍠楅悡娑樏归敐鍫綈鐎规洖鐬奸埀顒侇問閸ｎ噣宕滈悢闀愮箚闁割偅娲栭獮銏′繆閵堝拑姊楃紒鎲嬫嫹
	int light_draw = 0;
	int light_edit = 0;

	//闂傚倸鍊搁崐宄懊归崶褉鏋栭柡鍥ュ灩缁愭鏌熼悧鍫熺凡闁告垹濮撮埞鎴︽偐鐎圭姴顥濈紓浣哄閸ㄥ爼寮婚敐澶婄闁挎繂鎲涢幘缁樼厸闁告侗鍘介幉鍝ョ磼鏉堛劌娴€殿噮鍣ｉ獮瀣攽閸℃瑯鍟堥梻鍌欐缁鳖喚鈧潧鐭傚畷鎶芥晲婢跺﹥鐎梺褰掑亰閸ㄦ娊鎮块埀顒勬⒑閸濆嫬鈧綊顢栧▎鎰嚤闁硅崵绌﹊ght/width
	float maxAspectRatio = 30;
	float minAspectRatio = 1;

    float maxAngle = 45;
	float minAngle= 45;
};

//闁荤喍绀侀幊鎰板极閹惰棄绾ч柛顭戝枛濡﹢鏌℃笟濠冨
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
	* @brief 闂佸搫顑呯€氫即鍩€椤掑倸孝闁搞倝浜跺顐︽晸閿燂拷
	* @param 
	*/
	ArmorDetector();
	
	/**
	* @brief 闂佸搫顑嗛崝鏍倵椤栫偛绀勯柤鎭掑劜濞堬拷
	* @param
	*/
	~ArmorDetector();
	
	/**
	* @brief 闂備焦褰冮敃銉╁棘娓氣偓瀹曟﹢宕ㄩ鐘虫喖
	*/
	void release();
	
	/**
	* @brief 闂佽鍓濋褔宕ｉ崱娑樼煑闁挎繂鎳庡В鎰版煛娓氬﹥瀚�
	* @param 闂佸憡顭囬崰搴∶归敓锟� 闂佽桨绀侀張顒勫蓟閻旈潻绱ｆ繝濠傛椤ワ拷
	*/
	void runArmorDetector(const cv::Mat& srcImg, const EnemyColor enemy_color);
	
	/**
	* @brief	闁搞儲鍎抽崕姘紣閸曨偒妲遍柣鐑囨嫹
	* @param	闁告鍠庡ù锟� 闁轰礁鏈弻鐔革紣濠婂棗顥�
	* @return	
	*/
	cv::Mat imgPretreat(const cv::Mat & srcImg,const EnemyColor enemy_color);
	
	/**
	* @brief 闁哄嫷鍨伴幆渚€骞嶉幆褍鐓傞柣蹇ｅ灡濞硷拷
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
	* @brief 闁荤姳绶ょ槐鏇㈡偩缂佹鈻旈柕鍫濇搐娴狀垶姊婚崒婊庢缂佹劖绋撶划瀣晸閿燂拷
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