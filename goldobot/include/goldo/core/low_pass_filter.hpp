#pragma once

namespace goldobot
{
  class LowPassFilter
  {
  public:
    LowPassFilter();
    LowPassFilter(float _alpha);

    void reset();
    void set_alpha (float _new_alpha);
    float get_alpha () {return m_alpha;};
    float step (float _new_x);
    float out() {return m_out;}

  private:
    float m_alpha;
    float m_out;
    float m_x_n;
    float m_y_n;
    float m_y_n_1;
  };
}

