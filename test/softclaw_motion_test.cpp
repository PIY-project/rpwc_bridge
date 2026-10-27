#include <rpwc_bridge/softclaw_motion.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using Motion = rpwc_bridge::SoftClawMotion;

Motion::TimePoint at(double seconds)
{
  return Motion::TimePoint() + std::chrono::duration_cast<Motion::Clock::duration>(
                                   std::chrono::duration<double>(seconds));
}

void expect(Motion::Result actual, Motion::Result expected, const char* scenario)
{
  if (actual != expected)
  {
    std::cerr << "SoftClaw motion test failed: " << scenario << std::endl;
    std::exit(1);
  }
}

int main()
{
  const rpwc_bridge::SoftClawMotionConfig config;
  Motion motion(config);

  motion.start(0.0, 1.0, at(0.0));
  motion.observe(0.985, at(0.1));
  expect(motion.evaluate(at(0.3)), Motion::Result::PENDING,
         "elapsed time without new samples is not stable feedback");
  motion.observe(0.99, at(0.35));
  expect(motion.evaluate(at(0.35)), Motion::Result::TARGET_REACHED, "target reached");

  motion.start(0.0, 1.0, at(1.0));
  motion.observe(0.1, at(1.1));
  motion.observe(0.4, at(1.3));
  motion.observe(0.5, at(1.5));
  motion.observe(0.502, at(1.8));
  motion.observe(0.501, at(2.1));
  expect(motion.evaluate(at(2.1)), Motion::Result::EARLY_CLOSE, "object blocks full close");

  motion.start(0.1, 0.7, at(3.0));
  motion.observe(0.3, at(3.2));
  motion.observe(0.401, at(3.5));
  motion.observe(0.4, at(3.8));
  motion.observe(0.402, at(4.1));
  expect(motion.evaluate(at(4.1)), Motion::Result::EARLY_CLOSE, "partial close stabilizes");

  motion.start(0.8, 0.0, at(5.0));
  motion.observe(0.4, at(5.3));
  motion.observe(0.4, at(5.8));
  motion.observe(0.4, at(6.1));
  expect(motion.evaluate(at(6.1)), Motion::Result::PENDING, "opening cannot succeed early");
  expect(motion.evaluate(at(13.0)), Motion::Result::TIMEOUT, "incomplete opening times out");

  motion.start(0.2, 1.0, at(14.0));
  motion.observe(0.2, at(14.1));
  motion.observe(0.2, at(14.7));
  motion.observe(0.2, at(15.0));
  expect(motion.evaluate(at(22.0)), Motion::Result::NO_PROGRESS, "no closing progress");

  motion.start(0.0, 1.0, at(23.0));
  motion.observe(std::numeric_limits<double>::quiet_NaN(), at(23.1));
  expect(motion.evaluate(at(23.2)), Motion::Result::PENDING, "NaN feedback ignored");
  motion.reset();
  motion.start(0.5, 0.0, at(24.0));
  expect(motion.evaluate(at(24.3)), Motion::Result::PENDING, "old goal cannot finish new goal");

  std::cout << "SoftClaw motion tests passed" << std::endl;
  return 0;
}
