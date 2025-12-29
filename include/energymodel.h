class EnergyModel {
public:
  EnergyModel(double w_prime_base_, double ftp_base_, double tau_,
              double max_effort_base_);

  void reset();

  // call once per physics step
  void update(double power, double dt);
  void update2(double power, double dt);

  double get_wbal() const;
  double get_wbal_fraction() const;
  double get_ftp() const;
  double get_effort_limit() const;

  double get_tau() const { return tau; }

private:
  // parameters
  double ftp_base; // CP proxy
  double ftp;
  double w_prime_base;
  double w_prime; // total anaerobic capacity
  double tau;     // recovery time constant
  double max_effort;

  double effort_limit;

  void update_effort_limit();

  double calc_tau(double tau);

  // state
  double I = 0.0; // accumulated "fatigue integral"

private:
  double compute_tau(double dcp) const;
};
