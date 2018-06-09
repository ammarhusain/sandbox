// Copyright (C) 2016 Ammar Husain. All Rights Reserved.
/// \author: Ammar Husain (mrahusain@gmail.com)

#include <iostream> // std::cout
#include <Eigen/Dense>
#include <random>

typedef Eigen::Vector3f Point;

std::vector<Point>
stat_pts{{
Eigen::Vector3f(68.1319,0.908356,-0.529087), Eigen::Vector3f(68.1283,0.889887,-0.52878), Eigen::Vector3f(68.1265,0.871476,-0.528865), Eigen::Vector3f(68.1266,0.853099,-0.52934), Eigen::Vector3f(68.1306,0.834748,-0.530609), Eigen::Vector3f(68.1189,0.817227,-0.528634), Eigen::Vector3f(68.1149,0.798878,-0.528277), Eigen::Vector3f(68.1148,0.781456,-0.528692), Eigen::Vector3f(68.1205,0.76308,-0.530326), Eigen::Vector3f(68.1203,0.743806,-0.530766), Eigen::Vector3f(68.1023,0.725691,-0.527553), Eigen::Vector3f(68.1172,1.27819,-0.507003), Eigen::Vector3f(68.1051,1.25912,-0.504669), Eigen::Vector3f(68.1047,1.24117,-0.504742), Eigen::Vector3f(68.0983,1.22274,-0.503595), Eigen::Vector3f(68.1075,1.20652,-0.505643), Eigen::Vector3f(68.1048,1.18932,-0.505261), Eigen::Vector3f(68.1042,1.17141,-0.505297), Eigen::Vector3f(68.0995,1.15322,-0.504514), Eigen::Vector3f(68.0968,1.13432,-0.504146), Eigen::Vector3f(68.0979,1.11658,-0.504556), Eigen::Vector3f(68.097,1.09783,-0.504571), Eigen::Vector3f(68.0953,1.08079,-0.504097), Eigen::Vector3f(68.0922,1.06374,-0.503646), Eigen::Vector3f(68.093,1.04691,-0.503995), Eigen::Vector3f(68.0938,1.0292,-0.504354), Eigen::Vector3f(68.0984,1.01165,-0.505514), Eigen::Vector3f(68.0835,0.993293,-0.502611), Eigen::Vector3f(68.0821,0.97554,-0.502536), Eigen::Vector3f(68.0768,0.95679,-0.501655), Eigen::Vector3f(68.0851,0.939358,-0.503588), Eigen::Vector3f(68.0934,0.92187,-0.505514), Eigen::Vector3f(68.082,0.903899,-0.50337), Eigen::Vector3f(68.0744,0.886962,-0.502014), Eigen::Vector3f(68.0746,0.869315,-0.502287), Eigen::Vector3f(68.0846,0.85266,-0.50457), Eigen::Vector3f(68.0808,0.834943,-0.504012), Eigen::Vector3f(68.073,0.817239,-0.502632), Eigen::Vector3f(68.0691,0.799591,-0.502055), Eigen::Vector3f(68.0671,0.782849,-0.501863), Eigen::Vector3f(68.0748,0.765178,-0.503709), Eigen::Vector3f(68.0825,0.74745,-0.505545), Eigen::Vector3f(68.0665,0.72998,-0.502488), Eigen::Vector3f(68.2332,1.27834,-0.517944), Eigen::Vector3f(68.2284,1.25889,-0.516774), Eigen::Vector3f(68.2235,1.24037,-0.515573), Eigen::Vector3f(68.2198,1.22239,-0.514979), Eigen::Vector3f(68.2264,1.20493,-0.516202), Eigen::Vector3f(68.2284,1.18743,-0.516815), Eigen::Vector3f(68.2252,1.16917,-0.515976), Eigen::Vector3f(68.2212,1.15124,-0.515348), Eigen::Vector3f(68.2217,1.1342,-0.51529), Eigen::Vector3f(68.2214,1.11745,-0.51545), Eigen::Vector3f(68.2219,1.09954,-0.515384), Eigen::Vector3f(68.2177,1.08167,-0.514724), Eigen::Vector3f(68.2161,1.0628,-0.514238), Eigen::Vector3f(68.2097,1.04488,-0.513152), Eigen::Vector3f(68.2197,1.02839,-0.515074), Eigen::Vector3f(68.2172,1.01067,-0.51479), Eigen::Vector3f(68.2114,0.992585,-0.51343), Eigen::Vector3f(68.2068,0.974843,-0.512721), Eigen::Vector3f(68.2165,0.958246,-0.514603), Eigen::Vector3f(68.2117,0.940514,-0.513878), Eigen::Vector3f(68.2175,0.921973,-0.514937), Eigen::Vector3f(68.2028,0.904049,-0.512148), Eigen::Vector3f(68.2025,0.887169,-0.511942), Eigen::Vector3f(68.2093,0.869722,-0.513641), Eigen::Vector3f(68.2011,0.852743,-0.511776), Eigen::Vector3f(68.2104,0.835095,-0.513592), Eigen::Vector3f(68.2027,0.817393,-0.512054), Eigen::Vector3f(68.1968,0.799746,-0.510915), Eigen::Vector3f(68.1968,0.78213,-0.510997), Eigen::Vector3f(68.2006,0.76449,-0.511888), Eigen::Vector3f(68.2103,0.746765,-0.513999), Eigen::Vector3f(68.1983,0.729254,-0.511593), Eigen::Vector3f(68.1961,0.711663,-0.511227), Eigen::Vector3f(68.3694,1.25918,-0.525754), Eigen::Vector3f(68.369,1.24122,-0.525721), Eigen::Vector3f(68.353,1.22189,-0.522403), Eigen::Vector3f(68.3622,1.20483,-0.524397), Eigen::Vector3f(68.3577,1.18749,-0.523498), Eigen::Vector3f(68.3649,1.1702,-0.525064), Eigen::Vector3f(68.3545,1.15157,-0.522933), Eigen::Vector3f(68.3498,1.13343,-0.522021), Eigen::Vector3f(68.351,1.11573,-0.52233), Eigen::Vector3f(68.354,1.09813,-0.523038), Eigen::Vector3f(68.3609,1.08076,-0.524559), Eigen::Vector3f(68.3443,1.06202,-0.521144), Eigen::Vector3f(68.3471,1.04444,-0.521824), Eigen::Vector3f(68.346,1.02664,-0.521674), Eigen::Vector3f(68.339,1.00859,-0.520281), Eigen::Vector3f(68.3483,0.991409,-0.521901), Eigen::Vector3f(68.3431,0.974377,-0.52089), Eigen::Vector3f(68.3338,0.957241,-0.519047), Eigen::Vector3f(68.3403,0.939784,-0.520494), Eigen::Vector3f(68.331,0.921844,-0.518643), Eigen::Vector3f(68.3392,0.904404,-0.520483), Eigen::Vector3f(68.3298,0.886537,-0.518612), Eigen::Vector3f(68.3262,0.868843,-0.517965), Eigen::Vector3f(68.3244,0.851199,-0.517719), Eigen::Vector3f(68.3227,0.833569,-0.517465), Eigen::Vector3f(68.3247,0.815983,-0.518023), Eigen::Vector3f(68.3189,0.798354,-0.516926), Eigen::Vector3f(68.313,0.780769,-0.515819), Eigen::Vector3f(68.3188,0.763175,-0.517173), Eigen::Vector3f(68.3129,0.74563,-0.516046), Eigen::Vector3f(68.3225,0.728836,-0.518201), Eigen::Vector3f(68.3105,0.711405,-0.51582)

  }};



std::vector<Point> curb_pts{{
Eigen::Vector3f(56.1775,-11.8371,0.777236), Eigen::Vector3f(56.1734,-11.8475,0.778374), Eigen::Vector3f(56.1702,-11.8581,0.779233), Eigen::Vector3f(56.18,-11.8712,0.776447), Eigen::Vector3f(56.1477,-11.8435,0.774058), Eigen::Vector3f(56.1445,-11.854,0.774934), Eigen::Vector3f(56.1493,-11.8661,0.773537), Eigen::Vector3f(56.163,-11.88,0.769579), Eigen::Vector3f(56.1718,-11.893,0.767042), Eigen::Vector3f(56.1905,-11.9082,0.761657), Eigen::Vector3f(56.13,-11.8411,0.767599), Eigen::Vector3f(56.1229,-11.8507,0.769644), Eigen::Vector3f(56.1237,-11.8619,0.769386), Eigen::Vector3f(56.1284,-11.8739,0.767975), Eigen::Vector3f(56.1421,-11.8878,0.763968), Eigen::Vector3f(56.1558,-11.9019,0.759958), Eigen::Vector3f(56.1685,-11.9158,0.756235), Eigen::Vector3f(56.1783,-11.9292,0.753375), Eigen::Vector3f(56.188,-11.9426,0.750513), Eigen::Vector3f(56.0973,-11.8467,0.765653), Eigen::Vector3f(56.0902,-11.8561,0.767732), Eigen::Vector3f(56.0989,-11.8688,0.76514), Eigen::Vector3f(56.1126,-11.8827,0.761086), Eigen::Vector3f(56.1252,-11.8964,0.757321), Eigen::Vector3f(56.1349,-11.9096,0.754431), Eigen::Vector3f(56.1397,-11.9217,0.753002), Eigen::Vector3f(56.1445,-11.9338,0.751571), Eigen::Vector3f(56.1621,-11.949,0.746336), Eigen::Vector3f(56.0578,-11.8505,0.766005), Eigen::Vector3f(56.0626,-11.8623,0.764569), Eigen::Vector3f(56.0831,-11.8775,0.758396), Eigen::Vector3f(56.0918,-11.8902,0.755772), Eigen::Vector3f(56.1005,-11.9031,0.753146), Eigen::Vector3f(56.1013,-11.9142,0.75289), Eigen::Vector3f(56.1021,-11.9253,0.752633), Eigen::Vector3f(56.1197,-11.9403,0.747336), Eigen::Vector3f(56.1541,-11.9595,0.736994), Eigen::Vector3f(56.0303,-11.8566,0.763045), Eigen::Vector3f(56.0508,-11.8717,0.756798), Eigen::Vector3f(56.0555,-11.8834,0.755345), Eigen::Vector3f(56.0602,-11.8952,0.753893), Eigen::Vector3f(56.065,-11.9071,0.752439), Eigen::Vector3f(56.0658,-11.918,0.752185), Eigen::Vector3f(56.0744,-11.9309,0.749529), Eigen::Vector3f(56.095,-11.9466,0.743266), Eigen::Vector3f(56.1293,-11.9659,0.732792), Eigen::Vector3f(56.1983,-11.9941,0.711789), Eigen::Vector3f(56.003,-11.8519,0.760218), Eigen::Vector3f(56.0146,-11.865,0.756629), Eigen::Vector3f(56.0233,-11.8775,0.75395), Eigen::Vector3f(56.028,-11.8892,0.752484), Eigen::Vector3f(56.0288,-11.9,0.752234), Eigen::Vector3f(56.0335,-11.9118,0.750767), Eigen::Vector3f(56.0382,-11.9235,0.7493), Eigen::Vector3f(56.0468,-11.9363,0.746614), Eigen::Vector3f(56.0634,-11.951,0.741492), Eigen::Vector3f(56.1007,-11.9711,0.729973), Eigen::Vector3f(56.1853,-12.0035,0.703828), Eigen::Vector3f(55.9953,-11.8619,0.75148), Eigen::Vector3f(56.0039,-11.8744,0.748771), Eigen::Vector3f(56.0047,-11.8851,0.748522), Eigen::Vector3f(56.0054,-11.8959,0.748273), Eigen::Vector3f(56.0092,-11.9073,0.747101), Eigen::Vector3f(56.0139,-11.919,0.74562), Eigen::Vector3f(56.0146,-11.9297,0.745371), Eigen::Vector3f(56.0233,-11.9425,0.742656), Eigen::Vector3f(56.0359,-11.9562,0.738706), Eigen::Vector3f(56.0839,-11.9791,0.723651), Eigen::Vector3f(56.1989,-12.0196,0.687608), Eigen::Vector3f(55.9681,-11.8572,0.748936), Eigen::Vector3f(55.9728,-11.8686,0.747444), Eigen::Vector3f(55.9775,-11.8801,0.745952), Eigen::Vector3f(55.9783,-11.8908,0.745706), Eigen::Vector3f(55.9859,-11.903,0.743277), Eigen::Vector3f(55.9906,-11.9147,0.741782), Eigen::Vector3f(55.9874,-11.9244,0.742783), Eigen::Vector3f(55.9843,-11.9341,0.743784), Eigen::Vector3f(55.989,-11.9458,0.742288), Eigen::Vector3f(56.0084,-11.9612,0.736109), Eigen::Vector3f(56.0721,-11.9882,0.715869), Eigen::Vector3f(56.1948,-12.0313,0.676871), Eigen::Vector3f(55.935,-11.8614,0.748533), Eigen::Vector3f(55.9357,-11.8719,0.74829), Eigen::Vector3f(55.9443,-11.8842,0.745523), Eigen::Vector3f(55.948,-11.8954,0.744333), Eigen::Vector3f(55.9488,-11.9059,0.744089), Eigen::Vector3f(55.9495,-11.9164,0.743845), Eigen::Vector3f(55.9473,-11.9262,0.744549), Eigen::Vector3f(55.9481,-11.9368,0.744306), Eigen::Vector3f(55.9626,-11.9508,0.739637), Eigen::Vector3f(55.9898,-11.9682,0.730855), Eigen::Vector3f(56.0563,-11.9962,0.709416), Eigen::Vector3f(56.173,-12.0382,0.671828), Eigen::Vector3f(55.9257,-11.9426,0.740609), Eigen::Vector3f(55.9412,-11.9569,0.735567), Eigen::Vector3f(55.9713,-11.9752,0.72572), Eigen::Vector3f(56.0407,-12.0042,0.703057), Eigen::Vector3f(56.1561,-12.0463,0.665326), Eigen::Vector3f(55.9558,-11.9827,0.71973), Eigen::Vector3f(56.0289,-12.0131,0.695497), Eigen::Vector3f(56.1432,-12.0554,0.657629), Eigen::Vector3f(55.952,-11.9934,0.709899), Eigen::Vector3f(56.0446,-12.0295,0.678812), Eigen::Vector3f(56.1391,-12.0669,0.647049), Eigen::Vector3f(56.1878,-12.092,0.630698), Eigen::Vector3f(56.2102,-12.1099,0.623205), Eigen::Vector3f(55.956,-12.0063,0.697413), Eigen::Vector3f(56.0601,-12.0461,0.661969), Eigen::Vector3f(56.1312,-12.0773,0.637798), Eigen::Vector3f(56.1613,-12.0972,0.627567), Eigen::Vector3f(56.1709,-12.1113,0.62431), Eigen::Vector3f(56.1766,-12.1243,0.62238), Eigen::Vector3f(56.1774,-12.1359,0.622113), Eigen::Vector3f(56.187,-12.1501,0.618852), Eigen::Vector3f(56.1879,-12.1617,0.618584), Eigen::Vector3f(56.1838,-12.1718,0.61998), Eigen::Vector3f(56.0562,-12.0573,0.651689), Eigen::Vector3f(56.1144,-12.0851,0.631603), Eigen::Vector3f(56.1318,-12.1013,0.625626), Eigen::Vector3f(56.1365,-12.1139,0.624017), Eigen::Vector3f(56.1461,-12.128,0.620726), Eigen::Vector3f(56.1518,-12.1409,0.618778), Eigen::Vector3f(56.1565,-12.1536,0.617167), Eigen::Vector3f(56.1574,-12.1651,0.616901), Eigen::Vector3f(56.1582,-12.1765,0.616636), Eigen::Vector3f(56.0687,-12.0733,0.635629), Eigen::Vector3f(56.0977,-12.0928,0.625511), Eigen::Vector3f(56.1151,-12.109,0.619468), Eigen::Vector3f(56.1198,-12.1215,0.617844), Eigen::Vector3f(56.1255,-12.1344,0.61588), Eigen::Vector3f(56.1263,-12.1458,0.615617), Eigen::Vector3f(56.1271,-12.1571,0.615354), Eigen::Vector3f(56.1279,-12.1685,0.615091), Eigen::Vector3f(56.1239,-12.1783,0.616531), Eigen::Vector3f(55.9815,-12.0492,0.654761), Eigen::Vector3f(56.056,-12.0819,0.628377), Eigen::Vector3f(56.0811,-12.1003,0.619521), Eigen::Vector3f(56.0945,-12.1154,0.614788), Eigen::Vector3f(56.0992,-12.1279,0.613151), Eigen::Vector3f(56.0961,-12.1379,0.614267), Eigen::Vector3f(56.093,-12.1479,0.615384), Eigen::Vector3f(56.0939,-12.1592,0.615124), Eigen::Vector3f(56.0947,-12.1704,0.614864), Eigen::Vector3f(56.0955,-12.1817,0.614604), Eigen::Vector3f(55.9892,-12.0636,0.640576), Eigen::Vector3f(56.0365,-12.0885,0.62363), Eigen::Vector3f(56.0577,-12.1057,0.616069), Eigen::Vector3f(56.0624,-12.118,0.614419), Eigen::Vector3f(56.0593,-12.128,0.615554), Eigen::Vector3f(56.0552,-12.1375,0.617038), Eigen::Vector3f(56.056,-12.1486,0.616781), Eigen::Vector3f(56.0617,-12.1613,0.614782), Eigen::Vector3f(56.0702,-12.175,0.611737), Eigen::Vector3f(56.071,-12.1862,0.611479), Eigen::Vector3f(56.0132,-12.0936,0.620417), Eigen::Vector3f(56.0218,-12.107,0.617349), Eigen::Vector3f(56.0225,-12.118,0.617095), Eigen::Vector3f(56.0233,-12.129,0.616841), Eigen::Vector3f(56.0241,-12.1399,0.616587), Eigen::Vector3f(56.0327,-12.1535,0.613514), Eigen::Vector3f(56.0412,-12.1671,0.610441), Eigen::Vector3f(56.0459,-12.1794,0.608775), Eigen::Vector3f(56.0428,-12.1891,0.60993), Eigen::Vector3f(55.9908,-12.1095,0.61711), Eigen::Vector3f(55.9954,-12.1216,0.615435), Eigen::Vector3f(55.9962,-12.1325,0.615183), Eigen::Vector3f(56.0008,-12.1447,0.613507), Eigen::Vector3f(56.0055,-12.1568,0.61183), Eigen::Vector3f(56.0063,-12.1678,0.611578), Eigen::Vector3f(56.0032,-12.1773,0.612751), Eigen::Vector3f(55.9962,-12.1856,0.615351),


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
  // The planarity of the set of points is indicated by the smallest eigen value of the mean shifted
  // matrix of points. This is because the 2 largest eigen values correspond to the 2 orthogonal
  // axes on the plane, while the third smallest eigen value corresponds to how planar the points
  // are. A value of zero means that all points lie of the plane, while a very large eigen value
  // means that a lot of points are outliers. The value is multiplied by 1e5 to scale to a large
  // value for thresholding during the learning phase.
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
      (centered.adjoint() * centered) / double(mat_pts.rows() - 1);

  Eigen::EigenSolver<Eigen::MatrixXd> es(cov);
  std::cout << "eig vals: " << 1e5*es.eigenvalues() << std::endl;
  std::cout << "cov: " << 1e10*cov << std::endl;

  return cov;
}

int main(int argc, char **argv) {

  std::cout << "ED:   " << computeElevationDiff_(stat_pts) << std::endl;
  std::cout << "Pl:   " << computePlanarity_(stat_pts) << std::endl;
  std::cout << "CovD: " << computeCovariance_(stat_pts).determinant() << std::endl;
  std::cout << "--------" << std::endl;
  std::cout << "ED:   " << computeElevationDiff_(curb_pts) << std::endl;
  std::cout << "Pl:   " << computePlanarity_(curb_pts) << std::endl;
  std::cout << "CovD: " << 1e10*computeCovariance_(curb_pts).determinant() << std::endl;

  return 0;
}
