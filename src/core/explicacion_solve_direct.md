Perfecto. Aquí tienes el mismo hilo explicativo, ahora con un **paso 6 en C++ (Eigen)** y el antiguo paso 6 pasa a ser el **paso 7**.

---

## Reconstruir el movimiento del chasis a partir de las ruedas: mínimos cuadrados en una plataforma SWERV

En una plataforma *swerve*, cada módulo de rueda tiene orientación (\theta_i) y velocidad de rodadura (\alpha_i).
Si el chasis se mueve con (\mathbf{V}=[V_x,V_y]^T) y velocidad angular (\omega), la velocidad de contacto de la rueda (i), situada en (\mathbf{r}_i=(x_i,y_i)) (en `base_link`), es:
[
\mathbf{v}_i=
\begin{bmatrix}V_x\V_y\end{bmatrix}
+\omega,\hat{k}\times
\begin{bmatrix}x_i\y_i\end{bmatrix}
===================================

\begin{bmatrix}V_x-\omega,y_i\V_y+\omega,x_i\end{bmatrix}.
]

---

### 1) Del chasis a las ruedas (directa)

[
v_{ix}=V_x-\omega,y_i,\qquad
v_{iy}=V_y+\omega,x_i,
]
[
\theta_i=\operatorname{atan2}(v_{iy},v_{ix}),\quad
s_i=|\mathbf{v}_i|,\quad
\alpha_i=\frac{s_i}{R}.
]

### 2) Del par ((\alpha_i,\theta_i)) a ((v_{ix},v_{iy}))

[
v_{ix}=\alpha_i R\cos\theta_i,\qquad
v_{iy}=\alpha_i R\sin\theta_i.
]

### 3) Planteo del problema inverso

Conocidos ((\alpha_i,\theta_i)) y ((x_i,y_i)), queremos ((V_x,V_y,\omega)). Para 3 ruedas:
[
\underbrace{\begin{bmatrix}
1&0&-y_1\
0&1& x_1\
1&0&-y_2\
0&1& x_2\
1&0&-y_3\
0&1& x_3
\end{bmatrix}}*{A\in\mathbb{R}^{6\times3}}
\underbrace{\begin{bmatrix}V_x\V_y\\omega\end{bmatrix}}*{z}
===========================================================

\underbrace{\begin{bmatrix}
v_{1x}\v_{1y}\v_{2x}\v_{2y}\v_{3x}\v_{3y}
\end{bmatrix}}_{b}.
]

### 4) Por qué mínimos cuadrados

Son **6 ecuaciones** y **3 incógnitas**: sistema **sobredeterminado**. Si los datos fueran ideales, todas las ecuaciones se cruzan en el mismo punto. En la práctica hay pequeñas discrepancias. Mínimos cuadrados busca (z) que **minimiza (|Az-b|^2)**:
[
z=(A^\top A)^{-1}A^\top b\quad\text{o, numéricamente, resolviendo }A z \approx b.
]

### 5) Pasos prácticos (Python)

```python
import numpy as np

def wheels_to_chassis_ls(alphas, thetas, positions, R):
    """
    alphas, thetas: iterables de longitud N (rad/s y rad)
    positions: [(x1,y1),...,(xN,yN)] en base_link
    R: radio de rueda (m)
    Devuelve Vx, Vy, w
    """
    alphas = np.asarray(alphas, float)
    thetas = np.asarray(thetas, float)
    pos = np.asarray(positions, float)
    N = len(alphas)

    # 1) v_i en contacto
    s = alphas * R
    vix = s * np.cos(thetas)
    viy = s * np.sin(thetas)

    # 2) Construye A y b
    A = np.zeros((2*N, 3), float)
    b = np.zeros((2*N,), float)
    for i, (x, y) in enumerate(pos):
        A[2*i  ] = [1.0, 0.0, -y]
        A[2*i+1] = [0.0, 1.0,  x]
        b[2*i  ] = vix[i]
        b[2*i+1] = viy[i]

    # 3) Mínimos cuadrados
    z, *_ = np.linalg.lstsq(A, b, rcond=None)
    Vx, Vy, w = z
    return float(Vx), float(Vy), float(w)
```

### 6) Los mismos pasos en C++ con Eigen

```cpp
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <iostream>

struct ChassisVel { double Vx, Vy, w; };

ChassisVel wheels_to_chassis_ls_eigen(
    const std::array<double,3>& alpha,   // rad/s
    const std::array<double,3>& theta,   // rad
    const std::array<double,3>& x,       // m
    const std::array<double,3>& y,       // m
    double R)                             // m
{
    constexpr int N = 3;
    // 1) v_i en contacto
    std::array<double,N> vix, viy;
    for (int i = 0; i < N; ++i) {
        const double s = alpha[i] * R;
        vix[i] = s * std::cos(theta[i]);
        viy[i] = s * std::sin(theta[i]);
    }

    // 2) Construye A (6x3) y b (6)
    Eigen::Matrix<double, 2*N, 3> A;
    Eigen::Matrix<double, 2*N, 1> b;
    for (int i = 0; i < N; ++i) {
        A(2*i,   0) = 1.0;  A(2*i,   1) = 0.0;  A(2*i,   2) = -y[i];
        A(2*i+1, 0) = 0.0;  A(2*i+1, 1) = 1.0;  A(2*i+1, 2) =  x[i];
        b(2*i)      = vix[i];
        b(2*i+1)    = viy[i];
    }

    // 3) Mínimos cuadrados con QR (evita formar A^T A)
    Eigen::Vector3d z = A.colPivHouseholderQr().solve(b);

    return { z(0), z(1), z(2) }; // Vx, Vy, w
}

int main() {
    // Ejemplo mínimo (valores ilustrativos)
    std::array<double,3> alpha {6.1, 7.0, 5.5};   // rad/s
    std::array<double,3> theta {0.10, 1.20, -0.85}; // rad
    std::array<double,3> x     {0.35, -0.35, -0.35}; // m
    std::array<double,3> y     {0.00,  0.25, -0.25}; // m
    double R = 0.127; // m

    ChassisVel z = wheels_to_chassis_ls_eigen(alpha, theta, x, y, R);
    std::cout << "Vx=" << z.Vx << " m/s, Vy=" << z.Vy
              << " m/s, w=" << z.w << " rad/s\n";
    return 0;
}
```

**Notas C++/Eigen**

* Uso de `colPivHouseholderQr()` para resolver el **LS** sin formar (A^\top A).
* Si más adelante quieres **pesos**, basta con escalar filas de (A) y (b) por (\sqrt{w_i}) antes de llamar a `solve`.

### 7) Qué ganas con mínimos cuadrados

* Coherencia ante pequeñas discrepancias entre ruedas.
* Escala a más ruedas/medidas sin cambiar el planteamiento.
* Permite **ponderar** (dar menos peso a módulos menos fiables).
* Implementación compacta y numéricamente estable (QR/SVD).

Si quieres, lo empaqueto en un `.md` listo para tu repo (o en `.hpp/.cpp`).
