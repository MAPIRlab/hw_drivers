#ifndef URG_SERIAL_UTILS_H
#define URG_SERIAL_UTILS_H

/*!
  \file
  \brief �V���A���p�̕⏕�֐�

  \author Satofumi KAMIMURA

  $Id$
*/


//! �V���A���|�[�g����������
extern int urg_serial_find_port(void);


//! ���������V���A���|�[�g����Ԃ�
extern const char* urg_serial_port_name(int index);


/*!
  \brief �|�[�g�� URG ���ǂ���

  \retval 1 URG �̃|�[�g
  \retval 0 �s��
  \retval <0 �G���[
*/
extern int urg_serial_is_urg_port(int index);

#endif /* !URG_SERIAL_UTILS_H */
