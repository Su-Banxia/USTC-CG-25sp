$$
p=[s_{i},\ h_{1_i},\ h_{2_i},\ h_{3_i},\ t_{1_i},\ t_{2_i},\ t_{3_i}]^T
$$

$$
b_i=[v’_{k_x},\ v’_{k_y},\ v’_{k_z}],\ \ \ k \in \{i\}\ ∪\ N_i， N_i指V_i的邻点集合
$$

$$
A=\begin{bmatrix}
1 & -\frac{1}{d_1} & -\frac{1}{d_2}&...& 0\\
-\frac{1}{d_1}&1&0&...&0 \\
...&...&...&...&...\\
...&...&...&\frac{1}{d_n}&1
\end{bmatrix}\\
其每一行对应着v'_i-\frac{1}{d_i}\Sigma v'_j\\
其中v'_j是v'_i的邻点
$$

$$
x=\begin{bmatrix}
v'_{1_x} \\
v'_{1_y} \\
v'_{1_z} \\
... \\
v'_{n_x} \\
v'_{n_y} \\
v'_{n_z} \\
\end{bmatrix}
$$

$$
b=\begin{bmatrix}
-s_1 \delta_{1x} + h_{13} \delta_{1y} - h_{12} \delta_{1z}\\
-h_{13} \delta_{1x} - s_1 \delta_{1y} + h_{11} \delta_{1z}\\
h_{12} \delta_{ix} - h_{11} \delta_{1y} - s_1 \delta_{1z}\\
...\\
-s_n \delta_{nx} + h_{n3} \delta_{ny} - h_{n2} \delta_{nz}\\
-h_{n3} \delta_{nx} - s_n \delta_{ny} + h_{n1} \delta_{nz}\\
h_{n2} \delta_{nx} - h_{n1} \delta_{ny} - s_n \delta_{nz}\\


\end{bmatrix}
$$

$$
[s_{i},\ h_{1_i},\ h_{2_i},\ h_{3_i},\ t_{1_i},\ t_{2_i},\ t_{3_i}]^T=C_ib_i\\
b_i=[v’_{k_x},\ v’_{k_y},\ v’_{k_z},\ ...\ ]^T,\ \ \ k \in \{i\}\ ∪\ N_i， N_i指V_i的邻点集合
$$

现在的问题是向量b中的元素s，h是和未知数x中的分量是相关的，导致我无法通过A^T Ax=A^Tb求解方程，我应该怎么通过线性变换，修改对应的矩阵A，以满足需求？