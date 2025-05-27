#ifndef GEO_POS_CONV_HPP_
#define GEO_POS_CONV_HPP_

class geo_pos_conv
{
public:
  geo_pos_conv();

  double x() const;
  double y() const;
  double z() const;

  void set_plane(int num);
  void set_plane(double lat, double lon);
  void set_llh(double lat, double lon, double ele);
  void set_llh_nmea_degrees(double lat_d, double lon_d, double h);
  void set_xyz(double x, double y, double z);

  void llh_to_xyz(double lat, double lon, double ele);
  void conv_llh2xyz(void);
  void conv_xyz2llh(void);

private:
  double m_x, m_y, m_z;
  double m_lat, m_lon, m_h;
  double m_PLato, m_PLo;
};

#endif  // GEO_POS_CONV_HPP_
