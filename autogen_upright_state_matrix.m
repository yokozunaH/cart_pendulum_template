function A = autogen_upright_state_matrix(I_pend,b1,b2,g,m_cart,m_pend,r_com_pend)
%AUTOGEN_UPRIGHT_STATE_MATRIX
%    A = AUTOGEN_UPRIGHT_STATE_MATRIX(I_PEND,B1,B2,G,M_CART,M_PEND,R_COM_PEND)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    12-Apr-2020 22:34:27

t2 = r_com_pend.^2;
t3 = I_pend.*m_cart;
t4 = I_pend.*m_pend;
t5 = m_cart.*m_pend.*t2;
t6 = t3+t4+t5;
t7 = 1.0./t6;
t8 = m_cart+m_pend;
A = reshape([0.0,0.0,0.0,0.0,0.0,0.0,g.*m_pend.^2.*t2.*t7,g.*m_pend.*r_com_pend.*t7.*t8,1.0,0.0,-b1.*t7.*(I_pend+m_pend.*t2),-b1.*m_pend.*r_com_pend.*t7,0.0,1.0,-b2.*m_pend.*r_com_pend.*t7,-b2.*t7.*t8],[4,4]);
