#include <iostream>
#include "raisim/RaisimServer.hpp"
#include "final_exam_pr_STUDENTID.hpp"


#define MAKE_STR(x) _MAKE_STR(x)
#define _MAKE_STR(x) #x

int main() {
  std::string rsc_dir = std::string(MAKE_STR(RSC_DIR));
  /// create raisim world
  raisim::World world;
  world.setTimeStep(0.001);

  /// create objects
  world.addGround();
  auto chain = world.addArticulatedSystem(raisim::Path(rsc_dir + "\\chain\\robot.urdf").getString());

  /// cartPole state
  Eigen::VectorXd gc(chain->getGeneralizedCoordinateDim()), gv(chain->getDOF()), gf(chain->getDOF());

  gc << 1.0, 0, 0, 0, 0.7, 0.5, 0.4, 0.31622776601, 0.4, 0.3, 0.3, 0.81240384046;
  gv << 0.2, 0.3, 0.4,  0.3, 0.3, 0.4,  0.5, 0.5, 0.5;
  gf << 0.1, 0.2, 0.3,  0.5, 0.5, 0.3,  0.1, 0.2, 0.3;

  chain->setState(gc, gv);
  chain->setGeneralizedForce(gf);
  world.integrate1();

  /// launch raisim server
  raisim::RaisimServer server(&world);
  server.launchServer();
  server.focusOn(chain);

  Eigen::MatrixXd Minv = chain->getInverseMassMatrix().e();
  Eigen::VectorXd b = chain->getNonlinearities().e();
  Eigen::VectorXd acc_raisim = Minv * (gf-b);

  if((getMassMatrixUsingCRBA(gc) - chain->getMassMatrix().e()).norm() < 1e-8)
    std::cout<<"CRBA passed"<<std::endl;
  else
    std::cout<<"CRBA failed"<<std::endl;

  if((getNonlinearitiesUsingRNE(gc, gv) - chain->getNonlinearities().e()).norm() < 1e-8)
    std::cout<<"RNE passed "<<std::endl;
  else
    std::cout<<"RNE failed"<<std::endl;

  if ((getGaUsingABA(gc, gv, gf) - acc_raisim).norm() < 1e-8)
    std::cout<<"ABA passed"<<std::endl;
  else
    std::cout<<"ABA failed"<<std::endl;

  for (int i = 0; i < 200000000; i++) {
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }

  server.killServer();

  return 0;
}
