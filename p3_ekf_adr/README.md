En este README se explicarán cómo los diferentes niveles de ruido influyen en cada uno de los 3 modelos y las diferencias que hay entre ellos:

--Modelo 3D:

    Caso base (proc_noise_std = [0.002,0.002,0.001], obs_noise_std = [1.02,1.02,100.01]).

        El filtro pondera casi por igual su predicción (modelo) y la medición. Se ve que la trayectoria azul queda se aproxima bastante a la verde (ground truth) y la roja (observación), suavizando el ruido de esta sin dejar de incorporarla. Tiene un ligero retraso en incorporarse a maniobras muy bruscas, pero pocas oscilaciones.

    Alta incertidumbre en la observación (Q ×10: obs_noise_std = [10.2,10.2,1000.1]).
         El filtro considera que los datos de observación son muy ruidosos y se apoya mucho más en su modelo de movimiento. El filtro se separa de los datos de obserbación y se acerca al ground truth. Reacciona lento a picos en la observación. La trayectoria es más suave pero puede tener problemas si el efecto no es perfecto.

    Alta incertidumbre en el modelo de movimiento (R ×10: proc_noise_std = [0.02,0.02,0.01]).
        El filtro desconfía de su propia predicción y se fía casi exclusivamente de la observación. La trayectoria azul casi coincide con la roja y se pega de cerca a la verde, reacciona muy rápido a cada medición. Sigue muy fiel los datos de los sensores.

![Caso base](/p3_ekf_adr/Capturas/CasoBase3d.png)
![Alta incertidumbre en la observación 3D](/p3_ekf_adr/Capturas/RuidoAltoMedida3d.png)
![Alta incertidumbre en el modelo de movimiento 3D](/p3_ekf_adr/Capturas/RuidoAltoProceso3d.png)

--Modelo 3D:

    Caso base (proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1], obs_noise_std = [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]).

        Al igual que en el modelo 3D, el filtro pondera casi por igual su predicción y la medición. Suaviza bien la trayectoria sin  perderse grandes maniobras.

    Alta incertidumbre en la observación (Q ×10: obs_noise_std = [1000.0, 1000.0, 10000.0, 60.853891945200942e-06, 10.0966227112321507e-06, 0.15387262937311438, 0.15387262937311438]).

        El filtro considera desconfía de los sensores de posición y se apoya más en el modelo de velocidades y aceleraciones. Esto da mucha más flexibilidad para que el filtro desconfíe de la aceleración de la IMU pero mucho más en la velocidad odométrica, aunque no hay una gran diferencia respecto al modelo 3D.

    Alta incertidumbre en el modelo de movimiento (R ×10: proc_noise_std = [1, 1, 0.5, 1, 1, 1, 1]).

        El filtro desconfía de su propia predicción y se fía casi exclusivamente de la observación, obteniendo un seguimiento muy estricto pero sensible a picos de ruido.

    El EKF 7D suele ser más preciso y estable en entornos reales donde la dinámica no es tan ideal. Reacciona mejor a cambios rápidos de dinámica,
    y suaviza mejor las transiciones de velocidad o aceleración.

![Caso base](/p3_ekf_adr/Capturas/CasoBase7d.png)
![Alta incertidumbre en la observación 7D](/p3_ekf_adr/Capturas/RuidoAltoMedida7d.png)
![Alta incertidumbre en el modelo de movimiento 7D](/p3_ekf_adr/Capturas/RuidoAltoProceso7d.png)

--Modelo 8D:

    Caso base (proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1, 0.1], obs_noise_std = [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]).

        El filtro equilibra predicción y medición: la trayectoria azul queda “entre” la verde y la roja. Incorpora velocidades (vx,vy) y aceleraciones (ax,ay) suavizando cambios de dirección y velocidad.

    Alta incertidumbre en la observación (Q ×10: obs_noise_std = [1000.0, 1000.0, 10000.0, 60.853891945200942e-06, 10.0966227112321507e-06, 0.15387262937311438, 0.15387262937311438]).

        Prácticamente similar al modelo 7D

    Alta incertidumbre en el modelo de movimiento (R ×10: proc_noise_std = [1, 1, 0.5, 1, 1, 1, 1, 1]).

    El filtro desconfía de su propia predicción y se fía casi exclusivamente de la observación, obteniendo un seguimiento muy estricto pero sensible a picos de ruido.

        Muy parecido al modelo 7D


    El modelo de 7 estados originalmente sólo incorpora la velocidad lineal $v$ (como magnitud escalar), sin descomponerla en sus proyecciones vx, vy. Al ampliar el vector de estado para incluir explícitamente $v_x$ y $v_y$, logramos:
    -Una descripción más fiel del desplazamiento, pues capturamos cómo varían las velocidades en cada eje.
    -Una fusión más completa de las lecturas de acelerómetro y giroscopio, mejorando la precisión de la orientación y la dinámica.
    -Mayor capacidad para seguir trayectorias donde el robot no se mueve alineado con los ejes globales.  

![Caso base](/Capturas/CasoBase8d.png)
![Alta incertidumbre en la observación 38](/p3_ekf_adr/Capturas/RuidoAltoMedida8d.png)
![Alta incertidumbre en el modelo de movimiento 8D](/p3_ekf_adr/Capturas/RuidoAltoProceso8d.png)
    



    

