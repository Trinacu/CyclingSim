class EnergyModel {
public:
  EnergyModel(double w_prime_base_, double ftp_base_, double tau_);

  void reset();

  // call once per physics step
  void update(double power, double dt);

  double get_wbal() const;
  double get_wbal_fraction() const;
  double get_ftp() const;
  double get_effort_limit() const { return effort_limit; }

  double get_tau() const { return tau; }

private:
  // parameters
  double ftp_base; // CP proxy
  double ftp;
  double w_prime_base;
  double w_prime; // total anaerobic capacity
  double tau;     // recovery time constant

  double effort_limit;

  // state
  double I = 0.0; // accumulated "fatigue integral"

private:
  double compute_tau(double dcp) const;
};
