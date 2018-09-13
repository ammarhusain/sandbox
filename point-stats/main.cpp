// Copyright (C) 2016 Ammar Husain. All Rights Reserved.
/// \author: Ammar Husain (mrahusain@gmail.com)

#include <iostream> // std::cout
#include <Eigen/Dense>
#include <random>

typedef Eigen::Vector3f Point;
/*
std::vector<Point>
stat_pts{{
      Eigen::Vector3f(59.4643,9.02607,0.0736942), Eigen::Vector3f(59.4616,9.01559,0.0749675), Eigen::Vector3f(59.4665,9.00593,0.0736053), Eigen::Vector3f(59.4675,8.99586,0.0735608), Eigen::Vector3f(59.4648,8.98543,0.0748344), Eigen::Vector3f(59.4659,8.97538,0.07479), Eigen::Vector3f(59.4669,8.96533,0.0747456), Eigen::Vector3f(59.4318,9.0729,0.0728937), Eigen::Vector3f(59.4328,9.06293,0.0728497), Eigen::Vector3f(59.4377,9.05344,0.0714732), Eigen::Vector3f(59.4387,9.04346,0.0714291), Eigen::Vector3f(59.4398,9.03347,0.071385), Eigen::Vector3f(59.437,9.02306,0.0726737), Eigen::Vector3f(59.4353,9.01279,0.0736294), Eigen::Vector3f(59.4363,9.00283,0.0735854), Eigen::Vector3f(59.4345,8.9926,0.0745414), Eigen::Vector3f(59.4318,8.98229,0.0758306), Eigen::Vector3f(59.4328,8.97237,0.0757868), Eigen::Vector3f(59.4405,8.96301,0.0734096), Eigen::Vector3f(59.4444,8.95329,0.0723656), Eigen::Vector3f(59.4056,9.06952,0.071716), Eigen::Vector3f(59.4094,9.06002,0.0706617), Eigen::Vector3f(59.4105,9.05014,0.0706181), Eigen::Vector3f(59.4115,9.04025,0.0705744), Eigen::Vector3f(59.4097,9.03004,0.0715417), Eigen::Vector3f(59.4079,9.01985,0.0725091), Eigen::Vector3f(59.4052,9.00958,0.0738137), Eigen::Vector3f(59.4062,8.99974,0.0737702), Eigen::Vector3f(59.4111,8.99028,0.0723785), Eigen::Vector3f(59.4121,8.98042,0.072335), Eigen::Vector3f(59.4132,8.97056,0.0722915), Eigen::Vector3f(59.4142,8.96071,0.072248), Eigen::Vector3f(59.3832,9.06666,0.0693498), Eigen::Vector3f(59.3805,9.05637,0.0706694), Eigen::Vector3f(59.3787,9.04624,0.0716484), Eigen::Vector3f(59.3797,9.03648,0.0716054), Eigen::Vector3f(59.3779,9.02639,0.0725846), Eigen::Vector3f(59.3723,9.0159,0.0749271), Eigen::Vector3f(59.3705,9.00588,0.0759066), Eigen::Vector3f(59.3744,8.99647,0.0748413), Eigen::Vector3f(59.3792,8.98712,0.073435), Eigen::Vector3f(59.388,8.95841,0.0712606), Eigen::Vector3f(59.389,8.94865,0.0712175), Eigen::Vector3f(59.3929,8.93911,0.0701516), Eigen::Vector3f(59.3543,9.06291,0.069539), Eigen::Vector3f(59.3554,9.05323,0.0694963), Eigen::Vector3f(59.3536,9.0432,0.070487), Eigen::Vector3f(59.3546,9.03353,0.0704443), Eigen::Vector3f(59.3528,9.02353,0.0714354), Eigen::Vector3f(59.351,9.01356,0.0724266), Eigen::Vector3f(59.3454,9.00321,0.074796), Eigen::Vector3f(59.3436,8.9933,0.0757875), Eigen::Vector3f(59.3475,8.98398,0.0747111), Eigen::Vector3f(59.3551,8.97499,0.0722563), Eigen::Vector3f(59.3561,8.96535,0.0722137), Eigen::Vector3f(59.3533,8.95539,0.0735499), Eigen::Vector3f(59.3515,8.94554,0.0745414), Eigen::Vector3f(59.3554,8.93614,0.0734649), Eigen::Vector3f(59.3247,9.04936,0.0705795), Eigen::Vector3f(59.3285,9.04016,0.0694923), Eigen::Vector3f(59.3324,9.03093,0.0684051), Eigen::Vector3f(59.3334,9.02134,0.0683628), Eigen::Vector3f(59.3288,9.01112,0.0704106), Eigen::Vector3f(59.3241,9.00095,0.0724586), Eigen::Vector3f(59.3223,8.99112,0.0734617), Eigen::Vector3f(59.3205,8.98132,0.0744651), Eigen::Vector3f(59.3215,8.97181,0.0744231), Eigen::Vector3f(59.3197,8.96204,0.0754264), Eigen::Vector3f(59.3179,8.9523,0.0764299), Eigen::Vector3f(59.3189,8.94282,0.076388), Eigen::Vector3f(59.3199,8.93333,0.0763461), Eigen::Vector3f(59.3265,8.92426,0.0742131), Eigen::Vector3f(59.2951,9.04562,0.0715228), Eigen::Vector3f(59.3045,9.03725,0.0683121), Eigen::Vector3f(59.3111,9.02845,0.0661576), Eigen::Vector3f(59.3122,9.01894,0.0661156), Eigen::Vector3f(59.3104,9.00911,0.06713), Eigen::Vector3f(59.3057,8.999,0.0692012), Eigen::Vector3f(59.3011,8.98895,0.0712725), Eigen::Vector3f(59.4695,8.9957,0.0764958), Eigen::Vector3f(59.4727,8.96693,0.0763047), Eigen::Vector3f(59.471,8.95711,0.0772793), Eigen::Vector3f(59.4387,9.07928,0.074734), Eigen::Vector3f(59.4454,9.07051,0.0725736), Eigen::Vector3f(59.4577,9.06242,0.0683157), Eigen::Vector3f(59.4615,9.05318,0.0672032), Eigen::Vector3f(59.4626,9.04358,0.0671395), Eigen::Vector3f(59.4581,9.03332,0.0691736), Eigen::Vector3f(59.4479,9.02249,0.0733057), Eigen::Vector3f(59.4405,9.01207,0.0763896), Eigen::Vector3f(59.436,9.002,0.078425), Eigen::Vector3f(59.437,8.99254,0.0783622), Eigen::Vector3f(59.4437,8.98359,0.0762005), Eigen::Vector3f(59.4504,8.9746,0.0740387), Eigen::Vector3f(59.4515,8.96509,0.0739755), Eigen::Vector3f(59.4469,8.95512,0.0760114), Eigen::Vector3f(59.4148,9.07614,0.0735956), Eigen::Vector3f(59.4215,9.06747,0.0714129), Eigen::Vector3f(59.4309,9.0591,0.06817), Eigen::Vector3f(59.4376,9.05031,0.0659866), Eigen::Vector3f(59.4359,9.04045,0.0669838), Eigen::Vector3f(59.4341,9.03063,0.0679809), Eigen::Vector3f(59.4296,9.02051,0.0700387), Eigen::Vector3f(59.4222,9.01015,0.0731573), Eigen::Vector3f(59.4176,9.00015,0.0752156), Eigen::Vector3f(59.4131,8.9902,0.0772745), Eigen::Vector3f(59.4141,8.98083,0.0772122), Eigen::Vector3f(59.4152,8.97145,0.0771499), Eigen::Vector3f(59.4162,8.96207,0.0770876), Eigen::Vector3f(59.4145,8.95247,0.0780864), Eigen::Vector3f(59.4127,8.9429,0.0790853), Eigen::Vector3f(59.3966,9.07379,0.0704741), Eigen::Vector3f(59.4032,9.06517,0.0682692), Eigen::Vector3f(59.4071,9.05615,0.0671353), Eigen::Vector3f(59.4109,9.04709,0.0660014), Eigen::Vector3f(59.4103,9.02794,0.066948), Eigen::Vector3f(59.4113,9.01854,0.0668855), Eigen::Vector3f(59.3994,8.99831,0.0721205), Eigen::Vector3f(59.392,8.98815,0.0752746), Eigen::Vector3f(59.3847,8.97807,0.0784291), Eigen::Vector3f(59.3801,8.96831,0.0805121), Eigen::Vector3f(59.3811,8.95907,0.0804507), Eigen::Vector3f(59.3803,8.94969,0.0811043), Eigen::Vector3f(59.3813,8.94046,0.081043), Eigen::Vector3f(59.3795,9.06214,0.0674049), Eigen::Vector3f(59.3844,9.04389,0.0661985), Eigen::Vector3f(59.3826,9.03424,0.0672196), Eigen::Vector3f(59.3865,9.02526,0.0660748), Eigen::Vector3f(59.3875,9.01595,0.066013), Eigen::Vector3f(59.3858,9.00634,0.0670343), Eigen::Vector3f(59.3812,8.99646,0.0691394), Eigen::Vector3f(59.372,8.98619,0.0730504), Eigen::Vector3f(59.3646,8.97618,0.0762397), Eigen::Vector3f(59.3582,8.96633,0.0790683), Eigen::Vector3f(59.3564,8.95694,0.0800912), Eigen::Vector3f(59.3574,8.9478,0.0800305), Eigen::Vector3f(59.3584,8.93865,0.0799698), Eigen::Vector3f(59.3578,9.05002,0.0662745), Eigen::Vector3f(59.3588,9.04081,0.0662133), Eigen::Vector3f(59.3599,9.03159,0.0661521), Eigen::Vector3f(59.3637,9.0227,0.0649963), Eigen::Vector3f(59.3619,9.01316,0.0660297), Eigen::Vector3f(59.3602,9.00364,0.0670631), Eigen::Vector3f(59.3556,8.99387,0.0691917), Eigen::Vector3f(59.3539,8.98442,0.0702254), Eigen::Vector3f(59.3502,8.97482,0.0719895), Eigen::Vector3f(59.3457,8.96519,0.0741189), Eigen::Vector3f(59.3439,8.95584,0.0751535), Eigen::Vector3f(59.3421,8.94651,0.0761881), Eigen::Vector3f(59.3431,8.93742,0.0761277), Eigen::Vector3f(59.3423,8.92819,0.0767975), Eigen::Vector3f(59.3212,9.0546,0.0705779), Eigen::Vector3f(59.325,9.04589,0.0694121), Eigen::Vector3f(59.326,9.03681,0.0693518), Eigen::Vector3f(59.3288,9.02794,0.0685542), Eigen::Vector3f(59.3299,9.01885,0.0684938), Eigen::Vector3f(59.33,9.00047,0.0691104), Eigen::Vector3f(59.3311,8.99138,0.0690501), Eigen::Vector3f(59.3339,8.98248,0.0682522), Eigen::Vector3f(59.335,8.97338,0.0681918), Eigen::Vector3f(59.3341,8.96413,0.068869), Eigen::Vector3f(59.3296,8.95458,0.0710217), Eigen::Vector3f(59.3278,8.9453,0.0720682), Eigen::Vector3f(59.3288,8.93625,0.0720081), Eigen::Vector3f(59.3298,8.9272,0.071948), Eigen::Vector3f(59.2904,9.04153,0.0735299), Eigen::Vector3f(59.2914,9.03259,0.0734705), Eigen::Vector3f(59.2951,9.02398,0.0722939), Eigen::Vector3f(59.298,9.01523,0.0714896), Eigen::Vector3f(59.3018,9.00658,0.0703125), Eigen::Vector3f(59.3028,8.9976,0.0702529), Eigen::Vector3f(59.3066,8.9889,0.0690758), Eigen::Vector3f(59.3122,8.98036,0.0671532), Eigen::Vector3f(59.3132,8.97135,0.0670934), Eigen::Vector3f(59.3115,8.96209,0.0681513)
  }};



std::vector<Point> curb_pts{{
Eigen::Vector3f(58.454,7.58596,-0.0633309), Eigen::Vector3f(58.4613,7.60079,-0.0668364), Eigen::Vector3f(58.4542,7.60801,-0.069162), Eigen::Vector3f(58.4779,7.59579,-0.0666387), Eigen::Vector3f(58.4693,7.60407,-0.0679514), Eigen::Vector3f(58.4932,7.59185,-0.0652958), Eigen::Vector3f(58.4832,7.6012,-0.0656038), Eigen::Vector3f(58.4732,7.61049,-0.0658505), Eigen::Vector3f(58.4985,7.59729,-0.0642552), Eigen::Vector3f(58.4885,7.6066,-0.064569), Eigen::Vector3f(58.477,7.6169,-0.0637522), Eigen::Vector3f(58.5126,7.59444,-0.0617842), Eigen::Vector3f(58.5052,7.60169,-0.0642703), Eigen::Vector3f(58.4937,7.61201,-0.063534), Eigen::Vector3f(58.5221,7.5968,-0.0638848), Eigen::Vector3f(58.5147,7.60405,-0.0663927), Eigen::Vector3f(58.5074,7.61127,-0.0688639), Eigen::Vector3f(58.5331,7.59817,-0.0670259), Eigen::Vector3f(58.5271,7.6044,-0.0706154), Eigen::Vector3f(58.5169,7.61366,-0.0710012), Eigen::Vector3f(58.5353,7.60786,-0.0716694), Eigen::Vector3f(58.5523,7.60308,-0.0711908), Eigen::Vector3f(58.5449,7.61033,-0.0737702), Eigen::Vector3f(58.5641,7.60408,-0.0748291), Eigen::Vector3f(58.5796,7.60997,-0.067974), Eigen::Vector3f(58.5836,7.61654,-0.0658735), Eigen::Vector3f(58.5775,7.62275,-0.0695531), Eigen::Vector3f(58.3819,7.76037,0.0574016), Eigen::Vector3f(58.377,7.76547,0.0540475), Eigen::Vector3f(58.5793,7.63077,-0.0658918), Eigen::Vector3f(58.3887,7.76368,0.0567378), Eigen::Vector3f(58.3852,7.76781,0.0523147), Eigen::Vector3f(58.3948,7.7675,0.0566026), Eigen::Vector3f(58.3906,7.7721,0.0527013), Eigen::Vector3f(58.3828,7.77911,0.0514694), Eigen::Vector3f(58.5652,7.65839,-0.0461398), Eigen::Vector3f(58.4016,7.77084,0.0559449), Eigen::Vector3f(58.3945,7.77737,0.0541444), Eigen::Vector3f(58.3852,7.7853,0.0539733), Eigen::Vector3f(58.4274,7.75995,0.0488027), Eigen::Vector3f(58.41,7.77324,0.0542421), Eigen::Vector3f(58.3999,7.78167,0.054532), Eigen::Vector3f(58.4321,7.76482,0.049735), Eigen::Vector3f(58.406,7.7855,0.054394), Eigen::Vector3f(58.4597,7.75308,0.0417546), Eigen::Vector3f(58.4406,7.76731,0.0480674), Eigen::Vector3f(58.4107,7.79029,0.0553105), Eigen::Vector3f(58.524,7.718,0.00881978), Eigen::Vector3f(58.4608,7.76041,0.0452839), Eigen::Vector3f(58.4168,7.79414,0.0551759), Eigen::Vector3f(58.4604,7.76865,0.0498392), Eigen::Vector3f(58.4853,7.75898,0.0440905), Eigen::Vector3f(58.4629,7.77497,0.0523256), Eigen::Vector3f(58.4356,7.7959,0.0572833), Eigen::Vector3f(58.4788,7.77099,0.052718), Eigen::Vector3f(58.4677,7.77988,0.0532648), Eigen::Vector3f(58.4551,7.78964,0.0549143), Eigen::Vector3f(58.4813,7.77733,0.0551963), Eigen::Vector3f(58.4732,7.78432,0.0536898), Eigen::Vector3f(58.4606,7.79404,0.0553291), Eigen::Vector3f(58.4861,7.78226,0.05614), Eigen::Vector3f(58.4684,7.79708,0.0541951), Eigen::Vector3f(58.4858,7.79231,0.0535174), Eigen::Vector3f(58.4754,7.8006,0.0535817), Eigen::Vector3f(58.5011,7.78892,0.0544622), Eigen::Vector3f(58.4929,7.79587,0.0529223), Eigen::Vector3f(58.4824,7.80414,0.0529715), Eigen::Vector3f(58.4977,7.8008,0.0538659), Eigen::Vector3f(58.4642,7.58784,-0.0580632), Eigen::Vector3f(58.4586,7.59401,-0.0612028), Eigen::Vector3f(58.4512,7.60183,-0.0626399), Eigen::Vector3f(58.473,7.5897,-0.0603404), Eigen::Vector3f(58.4687,7.59476,-0.0646121), Eigen::Vector3f(58.457,7.60761,-0.0703094), Eigen::Vector3f(58.4812,7.59215,-0.0620585), Eigen::Vector3f(58.4875,7.59629,-0.0621164), Eigen::Vector3f(58.4777,7.6075,-0.06958), Eigen::Vector3f(58.5014,7.59262,-0.0605956), Eigen::Vector3f(58.4951,7.59934,-0.0632737), Eigen::Vector3f(58.4827,7.61273,-0.0685377), Eigen::Vector3f(58.5091,7.5957,-0.0617328), Eigen::Vector3f(58.5002,7.60461,-0.0622253), Eigen::Vector3f(58.4857,7.61959,-0.0658263), Eigen::Vector3f(58.5155,7.59989,-0.0617723), Eigen::Vector3f(58.5059,7.60932,-0.0617267), Eigen::Vector3f(58.4957,7.61925,-0.0610669), Eigen::Vector3f(58.4907,7.62481,-0.0647857), Eigen::Vector3f(58.5308,7.5952,-0.0612388), Eigen::Vector3f(58.5225,7.60356,-0.0623553), Eigen::Vector3f(58.5122,7.61351,-0.0617756), Eigen::Vector3f(58.502,7.6234,-0.0611287), Eigen::Vector3f(58.5386,7.59836,-0.0623464), Eigen::Vector3f(58.5289,7.60778,-0.0623893), Eigen::Vector3f(58.5173,7.61878,-0.0607255), Eigen::Vector3f(58.5097,7.6265,-0.062291), Eigen::Vector3f(58.5424,7.60477,-0.0601992), Eigen::Vector3f(58.534,7.61309,-0.0613314), Eigen::Vector3f(58.5237,7.62298,-0.0607711), Eigen::Vector3f(58.5579,7.60016,-0.0595647), Eigen::Vector3f(58.5488,7.60903,-0.0602188), Eigen::Vector3f(58.5404,7.61733,-0.0613619), Eigen::Vector3f(58.534,7.624,-0.0640976), Eigen::Vector3f(58.5644,7.60445,-0.0595705), Eigen::Vector3f(58.558,7.61117,-0.0623981), Eigen::Vector3f(58.5509,7.6184,-0.0646515), Eigen::Vector3f(58.3775,7.75994,0.0553932), Eigen::Vector3f(58.5736,7.60663,-0.0617214), Eigen::Vector3f(58.5665,7.61388,-0.0640303), Eigen::Vector3f(58.5614,7.61952,-0.0679313), Eigen::Vector3f(58.3913,7.75584,0.055558), Eigen::Vector3f(58.3839,7.76298,0.0546538), Eigen::Vector3f(58.374,7.77217,0.0560135), Eigen::Vector3f(58.5859,7.60476,-0.058298), Eigen::Vector3f(58.5815,7.6099,-0.0627923), Eigen::Vector3f(58.5757,7.61609,-0.0661959), Eigen::Vector3f(58.5685,7.6233,-0.06849), Eigen::Vector3f(58.397,7.75943,0.0553839), Eigen::Vector3f(58.3897,7.76655,0.0544697), Eigen::Vector3f(58.379,7.77623,0.0563712), Eigen::Vector3f(58.3697,7.78482,0.0572311), Eigen::Vector3f(58.5822,7.62043,-0.0661993), Eigen::Vector3f(58.5778,7.62554,-0.0706706), Eigen::Vector3f(58.4015,7.76407,0.0563083), Eigen::Vector3f(58.3941,7.77117,0.0553897), Eigen::Vector3f(58.3827,7.78132,0.0578383), Eigen::Vector3f(58.3728,7.7904,0.0592524), Eigen::Vector3f(58.4066,7.76819,0.0566852), Eigen::Vector3f(58.3972,7.77681,0.0574105), Eigen::Vector3f(58.3858,7.78692,0.0598576), Eigen::Vector3f(58.4199,7.76471,0.0574958), Eigen::Vector3f(58.4138,7.77079,0.0554232), Eigen::Vector3f(58.4037,7.7799,0.0566801), Eigen::Vector3f(58.3915,7.79048,0.0596631), Eigen::Vector3f(58.4264,7.76784,0.0567968), Eigen::Vector3f(58.4203,7.77391,0.054712), Eigen::Vector3f(58.4108,7.78249,0.055404), Eigen::Vector3f(58.3986,7.79305,0.0583663), Eigen::Vector3f(58.4336,7.77049,0.0555588), Eigen::Vector3f(58.4268,7.77706,0.0540043), Eigen::Vector3f(58.4173,7.78562,0.0546812), Eigen::Vector3f(58.403,7.79764,0.0592811), Eigen::Vector3f(58.4484,7.76606,0.0553939), Eigen::Vector3f(58.4395,7.77417,0.0554095), Eigen::Vector3f(58.4313,7.78173,0.0549352), Eigen::Vector3f(58.4211,7.79076,0.0561541), Eigen::Vector3f(58.4495,7.7733,0.0590259), Eigen::Vector3f(58.4426,7.77987,0.0574276), Eigen::Vector3f(58.4337,7.7879,0.0574991), Eigen::Vector3f(58.4241,7.79639,0.0581732), Eigen::Vector3f(58.4533,7.77851,0.0605014), Eigen::Vector3f(58.4471,7.78456,0.0583619), Eigen::Vector3f(58.4382,7.79256,0.0584281), Eigen::Vector3f(58.4279,7.80152,0.0596432), Eigen::Vector3f(58.4599,7.78172,0.0598277), Eigen::Vector3f(58.4516,7.78925,0.0592963), Eigen::Vector3f(58.4419,7.79772,0.0599004), Eigen::Vector3f(58.4848,7.77045,0.0522845), Eigen::Vector3f(58.4665,7.78495,0.0591574), Eigen::Vector3f(58.4575,7.79295,0.0591517), Eigen::Vector3f(58.4471,7.80189,0.0602861), Eigen::Vector3f(58.4844,7.77869,0.0569681), Eigen::Vector3f(58.4739,7.7877,0.0579549), Eigen::Vector3f(58.4641,7.79617,0.0584704), Eigen::Vector3f(58.4537,7.80509,0.0595891), Eigen::Vector3f(58.5031,7.77197,0.0543595), Eigen::Vector3f(58.4897,7.78297,0.0573857), Eigen::Vector3f(58.4784,7.79244,0.0588982), Eigen::Vector3f(58.4686,7.80088,0.0594078), Eigen::Vector3f(58.5056,7.77825,0.0569043), Eigen::Vector3f(58.4942,7.78774,0.0583361), Eigen::Vector3f(58.4837,7.79668,0.0593069), Eigen::Vector3f(58.4738,7.8051,0.0598074), Eigen::Vector3f(58.5024,7.79008,0.0566313), Eigen::Vector3f(58.4918,7.79901,0.0575805), Eigen::Vector3f(58.4798,7.80884,0.059671), Eigen::Vector3f(58.5006,7.80088,0.0553264), Eigen::Vector3f(58.4872,7.81164,0.0584639)
 }};
*/

std::vector<Point>
stat_pts{{
    Eigen::Vector3f(3,1, 2), Eigen::Vector3f(5,4, 2), Eigen::Vector3f(11,0, 2), Eigen::Vector3f(7,8, 2)
  }};

std::vector<Point>
curb_pts{{
    Eigen::Vector3f(3,1, 2), Eigen::Vector3f(5,4, 3), Eigen::Vector3f(11,0, 4), Eigen::Vector3f(7,8, 5)
  }};


double computeElevationDiff_(const std::vector<Point>& pts) {
  auto z_comparison = [](Point a, Point b) { return a(2) < b(2); };
  return std::abs((*std::max_element(pts.begin(), pts.end(), z_comparison))(2) -
                  (*std::min_element(pts.begin(), pts.end(), z_comparison))(2));
};

double computePlanarity_(const std::vector<Point>& pts) {
  Eigen::MatrixXd mat_pts(pts.size(), 3);
  for (size_t i = 0; i < pts.size(); ++i) {
    mat_pts.row(i) = pts[i].cast<double>();
  }
  Eigen::Matrix<double, 1, 3> mean = mat_pts.colwise().mean();
  const Eigen::MatrixXd points_centered = mat_pts.rowwise() - mean;
  int setting                           = Eigen::ComputeThinU | Eigen::ComputeThinV;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd = points_centered.jacobiSvd(setting);
  std::cout << 1e5*svd.singularValues()(0)*svd.singularValues()(1)*svd.singularValues()(2) << std::endl;
  std::cout << "sing vals: " << 1e5*svd.singularValues() << std::endl;
  return 1e5 * svd.singularValues()(2);
}

Eigen::MatrixXd computeCovariance_(const std::vector<Point> &pts) {
  Eigen::MatrixXd mat_pts(pts.size(), 3);
  for (size_t i = 0; i < pts.size(); ++i) {
    mat_pts.row(i) = pts[i].cast<double>();
  }
  Eigen::Matrix<double, 1, 3> mean = mat_pts.colwise().mean();
  const Eigen::MatrixXd centered = mat_pts.rowwise() - mean;

  Eigen::MatrixXd cov =
      (centered.transpose() * centered) / double(mat_pts.rows() - 1);

  Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
  std::cout << "eig vals: " << 1e5*es.eigenvalues() << std::endl;
 std::cout << "mean: " << mean << std::endl;
  std::cout << "cov: " << cov << std::endl;

  return cov;
}

Eigen::MatrixXd computeCovarianceOnline_(const std::vector<Point> &pts) {
  Eigen::Vector3d mean;
  Eigen::Matrix3d covariance;
for (int i = 0; i < pts.size(); ++i) {
  Eigen::Vector3d diff = pts[i].cast<double>() - mean;
  mean += diff / (i + 1);
  covariance += diff * diff.transpose() * i / (i + 1);
}

 std::cout << "mean: " << mean << std::endl;
  std::cout << "cov: " << covariance << std::endl;

  return covariance;
}
int main(int argc, char **argv) {

  std::cout << "ED:   " << computeElevationDiff_(stat_pts) << std::endl;
  std::cout << "Pl:   " << computePlanarity_(stat_pts) << std::endl;
  std::cout << "CovD: " << 1e10*computeCovariance_(stat_pts).determinant() << std::endl;
  std::cout << "CovOD: " << 1e10*computeCovarianceOnline_(stat_pts).determinant() << std::endl;
  std::cout << "--------" << std::endl;
  std::cout << "ED:   " << computeElevationDiff_(curb_pts) << std::endl;
  std::cout << "Pl:   " << computePlanarity_(curb_pts) << std::endl;
  std::cout << "CovD: " << 1e10*computeCovariance_(curb_pts).determinant() << std::endl;
  std::cout << "CovOD: " << 1e10*computeCovarianceOnline_(curb_pts).determinant() << std::endl;
  return 0;
}
