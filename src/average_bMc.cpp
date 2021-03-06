/*#include <ros/ros.h>
#include <mar_robot_arm5e/ARM5Arm.h>
#include <grasper/joint_offset.h>
#include <mar_ros_bridge/mar_params.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <tf/tfMessage.h>
#include <mar_perception/VirtualImage.h>
#include <std_srvs/Empty.h>*/
#include <iostream>
#include <list>
#include <visp/vpHomogeneousMatrix.h>
#include <tf/transform_broadcaster.h>


vpHomogeneousMatrix toMat( double a, double b, double c, double d, double e, double f, double g, double h, double i, double j, double k, double l, double mm, double n, double o, double p ){

  vpHomogeneousMatrix m(0, 0, 0, 0, 0, 0);

  m[0][0]=a;m[0][1]=b;m[0][2]=c;m[0][3]=d;
  m[1][0]=e;m[1][1]=f;m[1][2]=g;m[1][3]=h;
  m[2][0]=i;m[2][1]=j;m[2][2]=k;m[2][3]=l;
  m[3][0]=mm;m[3][1]=n;m[3][2]=o;m[3][3]=p;

  return m;

}

int main(int argc, char** argv){

  std::cout << "Computing average transform matrix." << std::endl;

  double mean_x, mean_y, mean_z, mean_c_roll, mean_s_roll, mean_c_pitch, mean_s_pitch, mean_c_yaw, mean_s_yaw;
  mean_x = mean_y = mean_z = 0.0;
  mean_c_roll = mean_s_roll = mean_c_pitch = mean_s_pitch = mean_c_yaw = mean_s_yaw = 0.0;


  std::list<vpHomogeneousMatrix> bMcs;	
  bMcs.push_back(toMat(
        0.1187839034, 0.8334158775, 0.5397298949, -0.2822248922, 
        -0.9898480561, 0.05666706589, 0.1303444262, -0.02619323719,  
        0.07804620483,  -0.549733407,  0.8316862216,  0.2556047074,  
        0,  0,  0,  1) ) ;
  bMcs.push_back(toMat(
        0.005762661922, 0.8282517135, 0.560326593, -0.2876063189, 
        -0.9977744165, 0.04198645717, -0.05180107249, 0.05847134769, 
        -0.06643045555, -0.5587810273, 0.8266503239, 0.2631088153, 
        0, 0, 0, 1 ) ) ;
  bMcs.push_back(toMat(
        0.09870474495, 0.8319954175, 0.5459313131, -0.284548422, 
        -0.9936340149, 0.05246453315, 0.09969411818, -0.01193329686, 
        0.054303018, -0.552296205, 0.8318774454, 0.2561452868, 
        0, 0, 0, 1 ) ) ;
  bMcs.push_back(toMat(
        0.01055128868, 0.8292595917, 0.558763993, -0.2869742353, 
        -0.9980841307, 0.04280283911, -0.0446764491, 0.05522464328, 
        -0.06096505923, -0.5572220801, 0.8281224638, 0.2623308288, 
        0, 0, 0, 1 ) ) ;
  bMcs.push_back(toMat(
        0.01323580436, 0.734056289, 0.6789596291, -0.3710095012, 
        -0.9988824599, 0.04051801942, -0.02433354422, 0.04961532202, 
        -0.04537229061, -0.6778787905, 0.7337722403, 0.2776605554, 
        0, 0, 0, 1 ) ) ;
  bMcs.push_back(toMat(
        0.006624740515, 0.7759620843, 0.6307447634, -0.3419287716, 
        -0.9989333986, 0.03395943672, -0.03128612695, 0.05271161518, 
        -0.04569658515, -0.6298647477, 0.7753594145, 0.2624120039, 
        0, 0, 0, 1) ) ;
  bMcs.push_back(toMat(
        0.01282732976, 0.7354921793, 0.6774117757, -0.3779047366, 
        -0.9989027838, 0.03994189831, -0.02445144743, 0.04820471542, 
        -0.04504096061, -0.6763548617, 0.7351975332, 0.2617725017, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        0.008056665977, 0.7743768667, 0.6326733426, -0.3430297907, 
        -0.9990110875, 0.03389901386, -0.02876984421, 0.05144557513, 
        -0.04372570423, -0.631815895, 0.7738841888, 0.2629897785, 
        0, 0, 0, 1 ) ) ;
  bMcs.push_back(toMat(
        -0.01240318675, 0.7881571785, 0.6153490253, -0.3221871372, 
        -0.998829571, 0.0190068416, -0.04447727587, 0.0616220273, 
        -0.04675092571, -0.6151804629, 0.7869989511, 0.2583913302, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        -0.03787216154, 0.7963293606, 0.6036764438, -0.3160685491, 
        -0.9969419227, 0.01121432577, -0.07733719427, 0.0842399059, 
        -0.06835570277, -0.6047592813, 0.7934694132, 0.2512278917, 
        0, 0, 0, 1) ) ; 
  bMcs.push_back(toMat(
        -0.01348828509, 0.78658925, 0.6173292622, -0.3238031839, 
        -0.9989082574, 0.01701522359, -0.04350603958, 0.0609566419, 
        -0.04472537846, -0.6172421194, 0.7855009908, 0.2587991193, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        -0.0356206843, 0.7967418077, 0.6032691428, -0.3157901515, 
        -0.997093818, 0.01234363024, -0.07517681109, 0.0828711142, 
        -0.0673430396, -0.6041937823, 0.7939866425, 0.2509849048, 
        0, 0, 0, 1 ) ) ;
  bMcs.push_back(toMat(
        -0.06887124986, 0.7822604115, 0.6191327803, -0.2904400467, 
        -0.9956534277, -0.01489351567, -0.09193712578, 0.09928084212, 
        -0.06269771008, -0.6227734996, 0.7798859951, 0.2856827431, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        -0.05677950504, 0.7833046743, 0.6190394777, -0.2948595819, 
        -0.9965383641, -0.006753051779, -0.08285942964, 0.09484032904, 
        -0.0607237729, -0.6216013059, 0.7809765937, 0.280827692, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        -0.04231349402, 0.7807850392, 0.6233652949, -0.2951317376, 
        -0.9979055434, -0.002472029006, -0.06464066465, 0.07866041844, 
        -0.04892948679, -0.6247948557, 0.779254319, 0.2850191433, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        -0.05651382544, 0.7823468088, 0.6202738575, -0.2959100416, 
        -0.9966343171, -0.00725465192, -0.08165419748, 0.0940279726, 
        -0.0593820299, -0.6228008034, 0.7801236657, 0.2813143095, 
        0, 0, 0, 1 ) ) ;
  bMcs.push_back(toMat(
        0.04924969109, 0.7601025718, 0.6479340617, -0.3263000743, 
        -0.9987355655, 0.04402968779, 0.02426225149, 0.02057774606, 
        -0.01008653469, -0.6483096999, 0.7613099204, 0.293156004, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        0.01129644989, 0.7775007985, 0.6287804851, -0.3159134906, 
        -0.9990661178, 0.03500236876, -0.02533232138, 0.04801219503, 
        -0.04170470652, -0.6279071129, 0.7771701069, 0.2845356361, 
        0, 0, 0, 1) ) ; 
  bMcs.push_back(toMat(
        0.04082687598, 0.7682088164, 0.6388962206, -0.3211161469, 
        -0.9990295388, 0.04196220796, 0.01338482949, 0.02639785326, 
        -0.01652715206, -0.6388226573, 0.7691764854, 0.2892682056, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        0.014748295, 0.7745833911, 0.6322998165, -0.3175892365, 
        -0.9991699153, 0.03543249133, -0.02010022191, 0.04524481467, 
        -0.03797325582, -0.6314785101, 0.7744629901, 0.286182111, 
        0, 0, 0, 1) ) ; 
  bMcs.push_back(toMat(
        0.05260897329, 0.838088133, 0.5429922443, -0.3069226984, 
        -0.9985887673, 0.04019553207, 0.03471012995, 0.02266164132, 
        0.007264285837, -0.5440520201, 0.8390200412, 0.2508357738, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        0.04119365198, 0.7951289322, 0.6050397212, -0.3256507022, 
        -0.9989360369, 0.0453409351, 0.008425784227, 0.03292826216, 
        -0.02073348192, -0.6047430701, 0.796150703, 0.2728834805, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        0.0518829428, 0.8371434095, 0.5445172837, -0.3072465774, 
        -0.9986349469, 0.04019703225, 0.03335328324, 0.02314782232, 
        0.006033502434, -0.5455044552, 0.8380862045, 0.2515228848, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat( 
        0.03953687727, 0.7950020448, 0.6053169287, -0.325834495, 
        -0.9989665, 0.04504252362, 0.006091218263, 0.03375133576, 
        -0.02242247108, -0.6049321614, 0.7959612509, 0.2728572659, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        0.07631494112, 0.8351345346, 0.5447259301, -0.2981198442, 
        -0.9953645496, 0.03174134155, 0.09078491361, -0.005583551505, 
        0.05852728477, -0.5491291254, 0.8336856485, 0.2470488744, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat( 
        0.06756259311, 0.8094494351, 0.583289729, -0.3132831694, 
        -0.9968431502, 0.03033014789, 0.07337449118, 0.002725485875, 
        0.04170167669, -0.5864057419, 0.8089433083, 0.2610869583, 
        0, 0, 0, 1 ) ) ;
  bMcs.push_back(toMat(
        0.06817147913, 0.8373957447, 0.5423292508, -0.2962371119, 
        -0.9965686398, 0.03157910572, 0.07650951783, 0.0008410573647, 
        0.04694247191, -0.5456840907, 0.8366751326, 0.2468457071, 
        0, 0, 0, 1) ) ; 
  bMcs.push_back(toMat( 
        0.05838217066, 0.8078918113, 0.5864318745, -0.3144432704, 
        -0.9978213755, 0.02914494758, 0.05918677829, 0.009043968346, 
        0.03072498728, -0.5886097122, 0.8078332636, 0.2618551401, 
        0, 0, 0, 1) ) ;
  bMcs.push_back(toMat(
        0.007727880449, 0.8468835075, 0.5317223002, -0.2913676267, 
        -0.995817565, 0.05492669692, -0.07300982983, 0.06712613405, 
        -0.0910365704, -0.528934195, 0.8437659392, 0.2451648882, 
        0, 0, 0, 1) ) ; 
  bMcs.push_back(toMat(
        -0.001020646531, 0.8474450887, 0.5308820773, -0.2904597598, 
        -0.993543984, 0.05936554982, -0.09667514322, 0.07748396345, 
        -0.1134429817, -0.5275533653, 0.8419133784, 0.2468786375, 
        0, 0, 0, 1) ) ;
  bMcs.push_back(toMat(
        0.006113649584, 0.8464976003, 0.5323574326, -0.2918220069, 
        -0.9957374113, 0.05414832701, -0.07466569796, 0.06791465637, 
        -0.09203059849, -0.5296317319, 0.8432203731, 0.2451380357, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        0.001043516417, 0.8439702302, 0.5363890022, -0.292826958, 
        -0.9939448938, 0.05981103317, -0.09217477132, 0.07550001906, 
        -0.1098747434, -0.5330449239, 0.8389223146, 0.2483640803, 
        0, 0, 0, 1) ) ; 
  bMcs.push_back(toMat(
        0.01394605961, 0.8774579489, 0.479450785, -0.2718948445, 
        -0.9925776887, 0.07008230014, -0.09938814412, 0.07879868318, 
        -0.1208099309, -0.4745060791, 0.8719225548, 0.223838753, 
        0, 0, 0, 1) ) ; 
  bMcs.push_back(toMat(
        0.01206081259, 0.876924386, 0.4804770109, -0.2719536235, 
        -0.9911966139, 0.07384383075, -0.1098924988, 0.08222571073, 
        -0.1318476751, -0.4749217934, 0.8700950987, 0.2252386651, 
        0, 0, 0, 1) ) ; 
  bMcs.push_back(toMat(
        0.01332508251, 0.8779729396, 0.4785247742, -0.2713657882, 
        -0.9926256671, 0.06928033021, -0.09947120572, 0.07885898768, 
        -0.1204853813, -0.4736705112, 0.872421641, 0.2237744276, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        0.01222184848, 0.8769204772, 0.4804800757, -0.2718807554, 
        -0.991279185, 0.07367917519, -0.1092563795, 0.08194355388, 
        -0.1312105321, -0.4749545829, 0.8701735117, 0.2253262037, 
        0, 0, 0, 1) ) ; 
  bMcs.push_back(toMat(
        -0.002867537858, 0.8696017011, 0.4937455405, -0.2592459631, 
        -0.9920588031, 0.05961134206, -0.1107511585, 0.07940205361, 
        -0.1257422302, -0.4901421931, 0.8625250849, 0.2250490371, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        -0.007267382675, 0.8722226736, 0.4890549998, -0.2593140622, 
        -0.991854138, 0.05590959964, -0.114452984, 0.08341554882, 
        -0.1271713569, -0.4859029989, 0.8647113516, 0.2171487442, 
        0, 0, 0, 1) ) ; 
  bMcs.push_back(toMat( 
        -0.003954980191, 0.8695479366, 0.4938327086, -0.259416463, 
        -0.9919845444, 0.05895920749, -0.1117607957, 0.0800928495, 
        -0.1262973544, -0.4903164262, 0.8623449313, 0.2248080277, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        -0.007593452098, 0.8723455704, 0.4888307942, -0.2591512776, 
        -0.9918762336, 0.05550454912, -0.1144586489, 0.08339767856, 
        -0.1269798282, -0.4857287832, 0.8648373676, 0.2170648302, 
        0, 0, 0, 1) ) ; 
  bMcs.push_back(toMat(
        -0.00609139794, 0.831274356, 0.5558289664, -0.2875286952, 
        -0.992292536, 0.06377138605, -0.1062484508, 0.08262820745, 
        -0.1237675961, -0.5521921363, 0.8244788819, 0.2469641425, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        -0.01246131662, 0.8355202261, 0.5493183662, -0.2862077597, 
        -0.9924473982, 0.05671258307, -0.1087742831, 0.08587040196, 
        -0.1220363771, -0.5465250541, 0.8285031611, 0.2392653348, 
        0, 0, 0, 1) ) ;
  bMcs.push_back(toMat(
        -0.002896957605, 0.8296398272, 0.5582914694, -0.2890256323, 
        -0.9926415623, 0.06519882907, -0.1020384314, 0.0799917952, 
        -0.1210550967, -0.5544789174, 0.823346096, 0.2478065477, 
        0, 0, 0, 1 ) ) ; 
  bMcs.push_back(toMat(
        -0.01485888143, 0.8358846826, 0.5487039375, -0.2855691801, 
        -0.9921688647, 0.05573774636, -0.1117776705, 0.08768616607, 
        -0.1240167635, -0.5460678539, 0.828510556, 0.2395782327, 
        0, 0, 0, 1) ) ; 
  bMcs.push_back(toMat(
        -0.01287793349, 0.8446257703, 0.5352022673, -0.2649747371, 
        -0.9843807867, 0.08327792193, -0.1551104589, 0.1094124154, 
        -0.1755808234, -0.5288403311, 0.8303609327, 0.2596663184, 
        0, 0, 0, 1) ) ;
  bMcs.push_back(toMat(
        -0.01688656969, 0.8484426853, 0.5290178197, -0.2604318728, 
        -0.9838427267, 0.08021748564, -0.1600582524, 0.1157542617, 
        -0.1782367328, -0.523173169, 0.8333795668, 0.2591041827, 
        0, 0, 0, 1 ) ) ;
  bMcs.push_back(toMat(
        -0.01098157088, 0.8410014091, 0.5409214684, -0.27141094, 
        -0.9846112349, 0.08526097605, -0.1525492773, 0.109357724, 
        -0.1744136495, -0.5342725857, 0.8271231365, 0.256988475, 
        0, 0, 0, 1) ) ;
  bMcs.push_back(toMat(
        -0.01172063883, 0.8404374848, 0.5417817465, -0.2727566998, 
        -0.98424413, 0.08590090338, -0.1545461981, 0.1135509275, 
        -0.1764259595, -0.5350568839, 0.8261888475, 0.2564501064, 
        0, 0, 0, 1) ) ;

  int total = 0;
  for (std::list<vpHomogeneousMatrix>::const_iterator iterator = bMcs.begin(), end = bMcs.end(); iterator != end; ++iterator) {
    //std::cout << *iterator;
    total ++;
    //tf::Transform pose;
    vpTranslationVector trans;
    iterator->extract(trans);

    vpQuaternionVector rot;
    iterator->extract(rot);
    tf::Quaternion rotation( rot.x(), rot.y(), rot.z(), rot.w());

    mean_x += trans[0];
    mean_y += trans[1];
    mean_z += trans[2];

    tf::Matrix3x3 m(rotation);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    mean_c_roll += cos(roll);
    mean_s_roll += sin(roll);
    mean_c_pitch += cos(pitch);
    mean_s_pitch += sin(pitch);
    mean_c_yaw += cos(yaw);
    mean_s_yaw += sin(yaw);
  }

  mean_x = mean_x / total;
  mean_y = mean_y / total;
  mean_z = mean_z / total;
  mean_c_roll = mean_c_roll / total;
  mean_s_roll = mean_s_roll / total;
  mean_c_pitch = mean_c_pitch / total;
  mean_s_pitch = mean_s_pitch / total;
  mean_c_yaw = mean_c_yaw / total;
  mean_s_yaw = mean_s_yaw / total;

  double mean_roll =  atan2(mean_s_roll, mean_c_roll);
  double mean_pitch =  atan2(mean_s_pitch, mean_c_pitch);
  double mean_yaw =  atan2(mean_s_yaw, mean_c_yaw);

  tf::Quaternion btQ;
  btQ = tf::createQuaternionFromRPY(mean_roll, mean_pitch, mean_yaw);
  vpTranslationVector tt(mean_x, mean_y, mean_z);
  vpQuaternionVector qq(btQ.x(), btQ.y(), btQ.z(), btQ.w());
  vpHomogeneousMatrix v(tt,qq);
  std::cout << "Average bMc " << std::endl << v << std::endl << "Decomposition" << std::endl;
    
    vpTranslationVector trans;
    v.extract(trans);
    std::cout << "X: " << trans[0] << " Y: " << trans[1] << " Z: " << trans[2];

    vpQuaternionVector rot;
    v.extract(rot);
    tf::Quaternion rotation( rot.x(), rot.y(), rot.z(), rot.w());
    tf::Matrix3x3 m(rotation);//quat
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
  
	std::cout << " Roll: " << roll << " Pitch: " << pitch << " Yaw: " << yaw << std::endl;
  
    std::cout << std::endl << "Formato Q -> " << "x: " << rot.x() << " y: " << rot.y() << " z: " 
    << rot.z() << " w: " << rot.w() <<std::endl;
  
    /* bMc found 
Computing average transform matrix.
Average bMc 
0.009921142609  0.8214341942  0.5702170074  -0.2980558034  
-0.9977100755  0.04628539033  -0.04931802806  0.06071263135  
-0.0669042314  -0.5684219623  0.8200123759  0.2551243192  
0  0  0  1  
Decomposition
X: -0.2980558034 Y: 0.06071263135 Z: 0.2551243192 Roll: -0.6061388669 Pitch: 0.06695424472 Yaw: -1.560852741

Formato Q -> x: -0.1894883746 y: 0.2325682007 z: -0.6640417613 w: 0.684875702
   */
}
