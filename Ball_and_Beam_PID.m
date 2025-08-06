%%
close all; clear all; clc;
s = tf('s');

% --- Parámetros del sistema físico ---
g = 9.8;
a1 = 1.3e-4;
a0 = 0.3;

% --- Planta interna: Motor DC (ángulo de la viga) ---
G1 = a0 / (s^2 + a1*s);

% --- Controlador PD (ángulo) ---
Kp_pd = 20;
Kd_pd = 0;
C_PD = Kp_pd + Kd_pd * s;

% --- Lazo cerrado interno (ángulo controlado) ---
G_inner = feedback(C_PD * G1, 1);

% --- Planta externa: Ángulo → Posición de la bola ---
G2 = -g / s^2;

% --- Planta total desde el PID externo ---
G_total = G2 * G_inner;

% --- Diseño de controlador PID externo (posición) ---
OS = 0.5;    % 10% sobreimpulso
Ts = 1;     % Tiempo de establecimiento deseado

zeta = -log(OS) / sqrt(pi^2 + log(OS)^2);
wn = 4 / (zeta * Ts);

% PID manual (puedes ajustar para cumplir tus criterios)
Kp = 0.1;
Ki = 0.001;
Kd = 0;

% ¡AQUÍ SE CORRIGE!
C_PID = -(Kp + Ki/s + Kd*s);  % Signo negativo compensa la planta negativa

% Lazo cerrado total
T_total = feedback(C_PID * G_total, 1);

% --- Simulación ---
t = 0:0.01:10;
ref = 10;
r = ref * ones(size(t));
[y, t_out] = lsim(T_total, r, t);

% --- Gráfica ---
figure;
plot(t_out, y, 'b', 'LineWidth', 2);
yline(ref, 'r--', 'Referencia');
xlabel('Tiempo [s]');
ylabel('Posición de la bola [m]');
title(sprintf('Respuesta al escalón - PID–PD (OS=%.1f%%, Ts=%.1fs)', OS*100, Ts));
legend('Salida del sistema', 'Referencia');
grid on;