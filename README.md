# Robot móvil con sistema de visión

  - [Introducción](#introducción)
  - [Montaje](#montaje)
  - [Programación del Piero-DIY](#programación-del-piero-diy)
    - [Calibración de los sensores](#calibración-de-los-sensores)
      - [Sensores de distancia Sonar](#sensores-de-distancia-sonar)
    - [Senalizacion usando LEDS](#senalizacion-usando-leds)
    - [Motores](#motores)
    - [Controlador reactivo](#controlador-reactivo)
      - [Alertas](#alertas)
    - [Encoders de los motores](#encoders-de-los-motores)
      - [Calibración de los encoders](#calibración-de-los-encoders)
    - [Control en Bucle Abierto](#control-en-bucle-abierto)
      - [Calibración de los motores (zona muerta)](#Calibracion-de-los-motores)
      - [Piero](#piero)
    - [Control en Bucle Cerrado](#control-en-bucle-cerrado)
      - [ControladorBC](#controladorbc)
    - [Cinemática del Piero](#cinemática-del-piero)
      - [PieroCV](#pierocv)
      - [MCI: Modelo Cinemático Inverso](#mci-modelo-cinemático-inverso)
      - [MCD: Modelo Cinemático Directo](#mcd-modelo-cinemático-directo)
      - [Odometría](#odometría)
      - [Control del modelo (Simulación o Hardware)](#control-del-modelo-simulación-o-hardware)
    - [Control de orientación](#control-de-orientación)
  - [Modelos para salir de clase](#modelos-para-salir-de-clase)
    - [Salir de clase con Signal Builder](#salir-de-clase-con-signal-builder)
    - [Salir de clase con Matlab Function](#salir-de-clase-con-matlab-function)
    - [Salir de clase con Waypoints](#salir-de-clase-con-waypoints)
    - [Salir de clase con StateFlow](#salir-de-clase-con-waypoints-y-evitando-obstaculos)
    - [Salir de clase con Waypoints y evitando obstaculos](#salir-de-clase-con-waypoints-y-evitando-obstaculos)
  - [Resumen de enlaces a todos los videos](#resumen-de-enlaces-a-todos-los-videos)

## Introducción

Este proyecto para la asignatura Laboratorio de Robótica en el curso 2023-24. Consta de un robot móvil al cual se le puede predefinir un camino o controlarse manualmente, además de poder ver el punto de vista del robot a través de una interfaz diseñada en NodeRed, usando una cámara con un sistema de movimiento que también puede ser manejado a través de NodeRed o de Telegram.

## Montaje

En primer lugar, se creó un banco de pruebas para ir probando parte de la programación de los motores, sensores, etc.

<img src="/fotos/im1.jpeg" alt="BancoDePruebas_Tapa" width="500" align="center">

Una vez se tuvo una primera toma de contacto con todos los elementos del banco de pruebas, se procedió al montaje final parcial del robot para su posterior programación completa.

<img src="/fotos/im5.jpeg" alt="Montaje 4" width="400" align="center">

Por último, realizamos todas las conexiones como se precisa en este esquema:


<img width="1131" alt="Captura de pantalla 2024-01-11 a las 20 21 14" src="/fotos/im6.png">


## Programación del Piero-DIY

### Calibración de los sensores
#### Sensores de distancia Sonar
El sensor de ultra sonidos UD-016 es capaz de medir distancias de entre 4 cm y 300 cm (con una precisión de 0.3cm +- 1%).
La salida analógica varía entre 0 y 5V y el rango de valores de salida del sensor varía entre 0 y 1015
Se toman las medidas de ambos sensores de forma experimental <br>
Sensor L:

Real (cm) | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 | 110 | 120 | 130 | 140 | 150 | 175 | 200 | 225 | 250
---|----|----|----|----|----|----|----|----|----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
Sensor | 32 | 69 | 99 | 130 | 162 | 190 | 228 | 253 | 293 | 330 | 360 | 392 | 430 | 462 | 497 | 580 | 678 | 756 | 836

Sensor R:

Real (cm) | 10 | 20 | 30 | 40 | 50 | 60 | 70 | 80 | 90 | 100 | 110 | 120 | 130 | 140 | 150 | 175 | 200 | 225 | 250
---|----|----|----|----|----|----|----|----|----|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----
Sensor | 29 | 66 | 96 | 130 | 160 | 192 | 225 | 260 | 294 | 326 | 360 | 395 | 427 | 463 | 497 | 578 | 665 | 746 | 830


Se representa la recta ideal de medidas (si el sensor fuese perfecto) con respecto a la obtenida experimentalmente. Se procede a realizar la calibración simultanea de los dos sensores. Para identificarlos, los distingueremos como sensor izquierdo ("L") y sensor derecho ("R").

```
distancia_teorica=[10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 175, 200, 225, 250];
distancia_medida_L = [32, 69, 99, 130, 162, 190, 228, 253, 293, 330, 360, 392, 430, 462, 497, 580, 678, 756, 836];
distancia_medida_R = [29, 66, 96, 130, 160, 192, 225, 260, 294, 326, 360, 395, 427, 463, 497, 578, 665, 746, 830];figure()
grid on
hold on
plot(distancia_teorica,distancia_teorica)
plot(distancia_teorica,distancia_medida_L)
plot(distancia_teorica,distancia_medida_R)
legend("Recta ideal", "Recta experimental (L)", "Recta experimental (R)",'Location','northwest')
hold off
```

<img src="/fotos/im7.png" alt="Gráfica rectas ideales vs experimentales">


Ahora, realizamos una regresión lineal para obtener la ecuacion de ajuste de los datos experimentales de cada sensor.

Y ajustaremos a una recta ya que es el polinomio que mejor se ajusta a simple vista
```
pol_L= polyfit(distancia_teorica,distancia_medida_L,1) 
pol_R = polyfit(distancia_teorica,distancia_medida_R,1)
```
Teniendo como resultados:
```
 pol_L =  [3.3698 -6.8969]
 pol_R =  [3.3396 -5.6439]
```
Representamos las rectas de ajuste con la ideal

```
x = 10:1:250;
y_teo = polyval([1 0],x);
y_exp_L = polyval(pol_L,x);
y_exp_R = polyval(pol_R,x);
figure()
grid on
hold on
plot(x,y_teo)
plot(x,y_exp_L)
plot(x,y_exp_R)
legend("Recta ideal","Recta experimental ajustada (L)", "Recta experimental ajustada (R)", 'Location','northwest')
hold off
```

<img src="/fotos/im8.png" alt="Gráfica rectas de regresión">
 
Para ajustar la recta del sensor a la teórica, realizaremos un ajuste de 0 y de ganancia:

```
cero_L = 0-polyval(pol_L,0)
cero_R = 0-polyval(pol_R,0)
pol_L(1,2) = pol_L(1,2)+cero_L;
pol_R(1,2) = pol_R(1,2)+cero_R;
K_L = 400/polyval(pol_L,400)
K_R = 400/polyval(pol_R,400)
```
Como resultado en la salida obtendremos una representación de la distancia medida en metros, a la que cada sensor detecta su obstáculo más cercano.

El diagrama implementado en SIMULINK es el siguiente:

<img src="/fotos/im9.png" alt="Modelo Sonars">

### Senalizacion usando LEDS
Para la implementación de un sistema de señalización adecuado que defina claramente los estados en los que se encuentra el Piero, haremos uso del siguiente diagrama: 
<table>
    <tr>
        <th>Verde</th>
        <th>Azul</th>
        <th>Amarillo</th>
        <th>Rojo</th>
        <th>Verde parpadeante</th>
    </tr>
    <tr>
        <td>Sin Obstaculo</td>
        <td>Obstaculo Derecha</td>
        <td>Obstaculo Izquierda</td>
        <td>Obstaculo Ambos Lados</td>
        <td>Obstaculo aproximandose</td>
    </tr>
</table>
<p>  
Esto lo implementaremos mediante el siguiente diagrama de SIMULINK, que será un Subsystem que tendrá como entrada un dato binario de 4 bits.
</p>
<h1></h1>
<img src="/fotos/im10.png" alt="Módulo Senalizacion">

<br>Colocamos para cada bit un bloque BitWise y un Boolean, los cuales nos permiten examinar y utilizar el valor de ese bit.
En el caso de la intermitencia, colocamos un bloque Switch para diferenciar los casos de Intermitencia (tren de pulsos) o Estable (bloque constante).

El video demostrativo de la señalización led de los obstáculos es el siguiente:

[![Video demostratvo Salir de clase con obstáculos](https://img.youtube.com/vi/zqo0bPWz1SM/0.jpg)](https://www.youtube.com/watch?v=zqo0bPWz1SM)

Este modelo se incluirá en los demás en forma de subsustema:

 <img src="/fotos/im11.png" alt="Módulo Senalizacion">

### Motores

Para el modelado de los motores se han creado dos módulos, uno para cada rueda. El diagrama utilizado es el siguiente:

<img src="/fotos/im12.png" alt="Modulo MotorsG31">

Cada uno de los circuitos cuenta con tres digital output: uno para ir hacia delante, otro que va hacia detrás y un enable que lo habilita.


### Controlador reactivo

El siguiente segmento presenta el desarrollo de un nuevo programa destinado a capacitar al robot Piero para detectar y evitar obstáculos mediante el uso de los sensores ubicados en la parte delantera. El programa actúa sobre los motores responsables del movimiento de las ruedas para modificar la dirección del robot y sortear obstáculos de manera efectiva.

<img src="/fotos/im13.png" alt="Modelo Evita obstáculos">


La implementación de este programa se basa en tres modelos trabajados previamente: el Modelado de Motores, el Modelado de Sensores y el Modelado de Señalización. Estos modelos proporcionan la base necesaria para que el Piero pueda reconocer obstáculos a través de sus sensores y tomar decisiones en tiempo real para evitar colisiones con objetos y paredes.

Este es un video demostrativo de el Piero evitando los obstáculos y con la señalización correspondiente:

[![Video demostratvo Salir de clase con obstáculos](https://img.youtube.com/vi/VB6QkytTeBs/0.jpg)](https://www.youtube.com/watch?v=VB6QkytTeBs)

### Encoders de los motores

Los motores de nuestro Piero cuentan con encoders que nos permiten medir la velocidad de giro del motor. Para ello primero debemos crear un bloque S-FunctionBuilder con la configuración que nos dice el fabricante y, sobre todo, asegurarnos de que los pines de los encoders izquierdo y derecho quedan declarados.

La primera ganancia es simplemente un ajuste para que los dos encoders midan lo mismo. La segunda ganancia realiza un cambio de unidades, multiplicando por "pi", por el radio de la rueda y dividido por el número de pulsos que nos da el encoder al dar una vuelta completa a la rueda. De esta manera conseguimos expresar dicha medida en metros. Por último se deriva la medida de la posición para obtener la velocidad.

<img src="/fotos/im14.png" alt="Modulo MotorsG31">





#### Calibración de los encoders

La calibración de los encoders se ha realizado en dos etapas:

 - Se han medido el número de interrupciones que se detectan en el motor al darle 10 vueltas completas a las ruedas y se ha dividido entre 10 para obtener el número de interrupciones por vuelta. Así obtenemos una media de las interrupciones por vuelta en lugar de medir sólo una vuelta puesto que conseguiremos un valor más real.

 - Se han realizado numerosas pruebas hasta conseguir que el robot vaya en línea recta y recorra la distancia exacta que le hemos indicado. Para ello, se modifican ligeramente las ganancias del número de interrupciones que se lee de cada rueda hasta conseguir una trayectoria lo más recta posible. Además para conseguir la distancia exacta se puede retocar (muy mínimamente) el número de interrupciones por vuelta. Esto se puede observar en las dós imágenes superiores. En la primera de ellas ya se ha ajustado los valores de las ganancias de cada rueda para que vaya en linea recta, sin embargo la distancia que recorre no es del todo precisa. Por eso se hace un modelo de prueba como el de la segunda imagen en el que el robot recorre la distancia de 10 baldosas de la clase del laboratorio que corresponde a $4,1 m$. Haciendo varias pruebas se queda ajustado hasta obtener un valor de 1496 interrupciones por vuelta.

Podemos ver el resultado de la calibración en el siguiente video:

[![Video demostratvo Calibración encoders](https://img.youtube.com/vi/kBWgP7cRCLU/0.jpg)](https://www.youtube.com/watch?v=kBWgP7cRCLU)



### Control en Bucle Abierto

Necesitamos que nuestro piero sea capaza de seguir una referencia de velocidad en cada una de las ruedas. Para ello se diseña un modelo de control en bucle abierto como el siguiente:

![image](/fotos/im15.png)

Su comportamiento es el que se espera y que analizamos a continuación:

![image](/fotos/im16.png)

![image](/fotos/im17.png)

Como se ovserva en la primera fotografía, la velocidad de las ruedas aumenta hasta conseguir el valor de la referencia y, si sufre perturbaciones, vuelve hasta ese valor. En la segunda captura se muestran dos perturbaciones de la rueda derecha y una de la rueda izquierda.

Un pequeño video demostrativo del piero con el control en bucle abierto nos ilustra como no corrrige la trayectoria si no la velodidad de la rueda:

[![Video demostratvo ControlBA](https://img.youtube.com/vi/OU3vdd7K3cA/0.jpg)](https://www.youtube.com/watch?v=OU3vdd7K3cA)

Como cabe de esperar, el piero andará en linea recta a $2 m/s$ y no reaccionará a las perturbaciones corrigiendo la trayectoria sino que volverá a la velocidad establecida en la referencia.

En este modelo podemos ver dos subsistemas:

#### ControladorBA
#### Calibracion de los motores
Es necesario comprobar cómo se comportan nuestros motores según la tensión de alimentación que les llega, por eso hemos realizado un sencillo test en el que aumentamos y disminuimos gradualmente la tensión y reflejamos el movimiento de las ruedas en una gráfica.

<img src="/fotos/im18.png" alt="">
La señal de referencia en este caso es una rampa ascendente y descendente. Al simular obtenemos el siguiente resultado:

<img src="/fotos/rampa.png" alt="">

Como podemos observar, exite un amplio rango de valores de tensión en los que no se produce movimiento en los motores. Este rango es mayor para el arranque que para la parada.

<img src="/fotos/im19.png" alt="">

Estos datos se almacenarán en una serie de archivos:


![image-1](/fotos/im20.png)

De estos datos, obtendremos las Look-Up Table

![image-2](/fotos/im21.png)


Mostramos un pequeño video demostrativo de como se han calibrado los motores en el que se observa también la zona muerta de los mismos:

[![Video demostratvo calibración de motores y zona muerta](https://img.youtube.com/vi/Tgh0nENfXSM/0.jpg)](https://www.youtube.com/watch?v=Tgh0nENfXSM)


#### Piero

Este subsistema se volverá a utilizar en cada uno de los modelos ya que se encarga de decidir si se simulará el modelo del piero o se enviará el control de PWM a los motores tomarán los valores leidos por los encoders. 
Esto se hace con un "Variador de fuente" que elige entre el piero real o Hardware (si se está monitoreando en tiempo real con el piero conectado al equipo) o bien con el modelo del comportamiento de nuestros motores.

Este modelo de comportamiento se obtiene de la siguiente manera:

Con el Piero conectado, creamos una señal de pulsos mediante signal builder y la simulamos, de manera que el robot realiza las acciones de arranque y parada varias veces. Esta gráfica la almacenamos en una tabla de datos para cada rueda. 

<img src="/fotos/im22.png" alt="">

<img src="/fotos/im23.png" alt="">

![image](/fotos/im24.png)

Obtenidos estos datos, los cargamos en "System Identification", un software que te permite analizar las funciones y elaborar funciones de transferencia ajustadas a nuestros datos.

<img src="/fotos/im25.png" alt="">


Una vez aquí, calculamos diferentes funciones de transferencia, en función del número de polos y ceros

<img src="/fotos/im26.png" alt="">


Como vemos, aquí se representan las gráficas elegidas, en base a lo bien que se ajustan con con el modelo de entrada.
Analizando los datos obtenidos llegamos a la siguiente conclusión:

La que mejor se ajusta, es la de dos polos y un cero. Podríamos haber elegido esta pero el resultado sería muy sensible a alteraciones. Nosotros queremos un sistema estable y gradual, que no cambie bruscamente, por eso elegiremos la TFi10, de un polo. Esta nos dará buenos resultados y como observaremos más adelante en los videos, tenemos un control estable.


### Control en Bucle Cerrado

Para el control de la velocidad en el bucle cerrado haremos algo parecido pero, por supuesto cerrando el bucle.

El modelo que usaremos para llevar a cabo este comportamiento será el siguiente:

![image](/fotos/im27.png)

En este caso, el piero andará en linea recta a $2 m/s$ pero sí reaccionará a las perturbaciones corrigiendo la trayectoria y volverá a la velocidad lineal del robot establecida en la referencia ya que acumula el posible error en cada rueda.

Así, probando los resultados, vemos que si lo ejecutamos y nos oponemos al movimiento de una rueda, el PID hace que se recupere y continúe en la dirección que iba.


<img width="830" alt="Captura de pantalla 2024-01-11 a las 12 33 06" src="/fotos/im28.png">



En el siguiente vídeo se ilustra este funcionamiento

[![Video demostratvo ControlBC](https://img.youtube.com/vi/-JRCHIIsw-Q/0.jpg)](https://www.youtube.com/watch?v=-JRCHIIsw-Q)

Los subsistemas que se utilizan en este caso son el mismo que anteriormente para el modelo del piero y el controlador en bucle cerrrado:

#### ControladorBC

El controlador en este caso lo haremos con un PID para cada rueda en lugar de las look-up tables.


<img width="1329" alt="Captura de pantalla 2024-01-11 a las 12 38 49" src="/fotos/im29.png">


Utilizo el PID tunner para establecer un tiempo de establecimiento de $1 segundo$ aproximadamente ya que así será suficientemente rápido y el tiempo de subida puede ser del orden de medio segundo, la sobreoscilación será $0%$. Hago lo mismo en los dos PID.


<img width="1110" alt="Captura de pantalla 2024-01-11 a las 12 40 32" src="/fotos/im30.png">


Una vez se ha probado el modelo de forma experimental y con cambios en la orientación, hemos observado que el controlador tenía cierta sobreoscilación al intentar alcanzar una trayectoria dada. Por tanto se ha vuelto a ajustar, con el PID tunner, reduciendo un poco la ganancia derivativa entre otras cosas y se concluye usando el controlador PID con los parámetros que aparecen en la siguiente captura para ambas ruedas puesto que los resultados son muy buenos.

![image](/fotos/im31.png)


### Cinemática del Piero

A continuación se describe el modelo de la cinemática a implementar en el Piero DIY. Será el que aparece en la siguiente imagen y se describe a continuación:

![image](/fotos/im32.png)

Como entrada, en el esquema se están metiendo 2 constantes que son la velocidad lineal en $m/s$ y la velocidad angular en $rad/s$.
Esto es lo que se trata de lograr con este modelo: conseguir que velocidades lineales y angulares sean interpretadas y reproducidas en nuestro piero que sólo entiende de velocidad de las ruedas por separado

Para lograr esto, tendremos que aplicar un modelo cinematico tanto antes como despues del control de velocidad de las ruedas.

Por tanto, los subsistemas que se han utilizado en este caso son:

#### PieroCV
Se trata de un control de velocidad como el que ya hemos hemos implementado anteriormente, usando el bucle cerrado para poder reaccionar ante perturbaciones.
El subsistema es:
![image](/fotos/im33.png)

Y toma la referencia de velocidad de la rueda izquierda y derecha para realizar el control.

#### MCI: Modelo Cinemático Inverso
Antes del control le aplicaremos el jacobiano inverso para convertir la velocidad cartesiana local en la velocidad de avance de las ruedas.
![image](/fotos/im34.png)

#### MCD: Modelo Cinemático Directo
Después del control le aplicaremos un modelo cinemático con el jacobiano directo para convertir la velocidad de las ruedas en velocidad cartesiana local.
![image](/fotos/mcd.png)

Nota: Para la matriz de los jacobianos necesitamos un parámetro del piero, la distancia entre los ejes de las ruedas que, medida manualmente, establecemos que es $d = 0.22 m$

#### Odometría
Por último, calculamos la odometría de tal forma que, por ejemplo para la rotación de un ángulo $\alpha$ sobre el eje $Z$ se hará:

$$\dot{X} = V_x \cdot cos\left(\alpha\right) - V_y \cdot sen\left(\alpha\right)$$

$$\dot{Y} = V_x \cdot sen\left(\alpha\right) - V_y \cdot cos\left(\alpha\right)$$


Y nos quedaría algo así:

<img width="874" alt="Captura de pantalla 2024-01-11 a las 12 41 13" src="/fotos/im35.png">

#### Control del modelo (Simulación o Hardware)

Si ponemos como constantes de entrada:
  - Velocidad lineal: $0.2 m/s$
  - Velocidad angular: $\frac{\pi}{5} rad/s$

Y hacemos la simulación (durante unos 5 segundos), monitoreando el robot real, obtenemos lo que esperamos, el robot anda hacia adelante y gira a la izquierda, describiendo la siguiente trayectoria con la odometría:
![image](/fotos/im36.png)

También mostramos un video del comportamiento en el robot real

[![Video demostratvo PieroCinematica](https://img.youtube.com/vi/YsWtWxGMNfY/0.jpg)](https://www.youtube.com/watch?v=YsWtWxGMNfY)

### Control de orientación

Por último, para conseguir que nuestro piero sea completamente controlable, tenemos que ser capaces de controlar la orientación.

Añadiremos, entonces, un control de orientación para que, en lugar de tener la velocidad lineal y angular como constantes, la velocidad angular esté controlada mediante un controlador (PID) que actua sobre el error en la orientación medida del modelo real del Piero. El modelo sería similar al anterior pero con este controlador para lograr el cambio de orientación

<img width="1438" alt="Captura de pantalla 2024-01-11 a las 12 42 05" src="/fotos/im37.png">

Es importante que el PID tenga límites en la saturación. En este caso le hemos puesto el límite superior en 3 y el inferior en -3. Estos valores dependen de los motores y de la velocidad máxima que querremos. En este caso se limitará a unos 0.3 m/s

Por ejemplo, si ejecutamos la simulación en el piero y monitoreamos un cambio de orientación de $0$ a $\frac{\pi}{2} rad$, observamos que el controlador actúa hasta que se ha logrado establecer el ángulo en $90^o$.

![image](/fotos/im38.png)

Teniendo este modelo, vamos a ver si podemos corregir algo del posible error que se puede cometer por la medida de la distancia entre ejes de forma manual (que establecimos previamente en $d = 0.22 m$).

Para ello, haremos que el Piero de 10 vueltas sobre sí mismo (para ver el error acumulado ya que, en una sola vuelta, no sería suficiente error) poniendo la velocidad lineal a $v = 0 m/s$ y lel error en orientación a $20\pi$ que equivaldría a 10 vueltas completas.

Si en la realidad da menos de 10 vueltas, hay que bajar la d porque el sistema cree que ha recorrido más espacio cuando no lo ha hecho y si se para antes de tiempo, hay que subir ligeramente la d
En nuestro caso, como se observa en el siguiente video se ha quedado corto, le ha faltado más de una vuelta, así que habría que aumetar el valor de la distancia entre ruedas. 

[![Video demostratvo 10 Vueltas Incompletas](https://img.youtube.com/vi/_kR8HMQxs_Y/0.jpg)](https://www.youtube.com/watch?v=_kR8HMQxs_Y)


Tras probar varias veces, acabamos dando las 10 vueltas exactas (como se aprecia en el vídeo y finalmente el valor que tomamos es de $d = 0.2275 m$

[![Video demostratvo 10 Vueltas Completas](https://img.youtube.com/vi/Sdw2mhJyUDc/0.jpg)](https://www.youtube.com/watch?v=Sdw2mhJyUDc)


Vemos entonces, que se ha corregido el error en la orientación haciendo el mismo giro de 90 grados.

![image](/fotos/im39.png)
 


## Modelos para salir de clase
A continuación se detallarán una serie de modelos que proporcionarán al Piero las actuaciones necesarias para conseguir seguir una trayectoria predefinida. Han sido creados para comprobar la versatilidad que ofrece SIMULINK para realizar dicha tarea y así poder comparar para ver qué modelo se adapta mejor.

### Salir de clase con Signal Builder

<img src="/fotos/im40.png" alt="SignalBuilder">

Este modelo es el primero que se implementó, y el más rudimentario. 
Directamente a través de un generador de señales (Signal Builder), se crea una para la velocidad lineal y otra para la angular. Esta gráfica representa el valor de salida que estará enviando a lo largo del tiempo.
Es muy complicado definir una trayectoria precisa mediante este método, ya que habrá que calcular para cada movimiento, el tiempo ddurante el que se tiene que aplicar cada comando para recorrer la distancia o girar el ángulo deseado.

### Salir de clase con Matlab Function


<img src="/fotos/im41.png" alt="waypoints">

El modelo Salir de clase con Matlaf Function hace uso del bloque Matlab function, al que se le ha programado un pequeño algoritmo de Pure Pursuit.

<img src="/fotos/im42.png" alt="MF">

Como podemos ver en esta imagen ampliada, la trayectoria pasa directamente a la variable "camino", la posición x e y vienen de la odometría, y la i, representa la iteración por donde va respecto a la trayectoria.


<img src="/fotos/im43.png" alt="MF">

Este bloque tomaraá como entrada la matriz de puntos predefinida y la posición X,Y del robot. Como salida, se encarga de proporcionar valores de velocidad lineal y angular adecuados para seguir la trayectoria.

Hemos hecho un par de intentos y no funciona mal pero hemos obtenido algunos resultados mejores usando el bloque de Pure Pursuit. No obstante, se adjunta un video de prueba con resultasdos muy razonables:

[![Video demostratvo Salir de clase usando Matlab Function](https://img.youtube.com/vi/nUrU1EKVeNY/0.jpg)](https://www.youtube.com/watch?v=nUrU1EKVeNY)

### Salir de clase con Waypoints

<img src="/fotos/im44.png" alt="waypoints">
El modelo de Salir de clase con Waypoints hace uso del bloque Pure Pursuit. Este algoritmo es muy utilizados en sistemas de conducción autónoma, donde se proporciona al robot comandos de velocidad lineal y angular para perseguir una trayectoria predefinida con puntos x e y.

<img src="/fotos/im45.png" alt="XY">

Como podemos observar en la simulación, el modelo es preciso.

### Salir de clase con StateFlow

El objetivo de este modelo es que el PIERO salga de clase por sí mismo a partir de unas consignas de velocidad lineal y velocidad angular deseada, en este caso la principal diferencia respecto al resto de salir de clase es el uso de la herramienta State Flow.

Stateflow es una herramienta de modelado de sistemas dinámicos desarrollada por MathWorks. Permite al usuario diseñar, simular y analizar sistemas de tiempo discreto y continuo mediante el uso de diagramas de estados y transiciones, funciones y tablas de eventos. Es utilizado principalmente en aplicaciones de control, automatización, procesamiento de señales y análisis de sistemas complejos. Stateflow se integra con otros productos de MathWorks, como Simulink, para proporcionar una solución completa de modelado y simulación.

<img src="/fotos/im46.png" alt="waypoints">

A la entrada del diagrama tenemos la odometría de nuestro robot; y, sabiendo su posición, podemos enviar comandos de velocidad para cubrir distancias específicas tanto lineales como angulares. De esta manera programamos la trayectoria completa deseada.

  

Todos los videos de salir de clase con los diferentes métodos devuelven resultados muy similares pero, después de probarlos todos y ajustarlos para que funcionen lo mejor posible, el que mejor resultados nos ha dado es en el que mayor control de la trayectoria y, por tanto, mejores resultados nos ha dado ha sido en el que usamos el bloque "Pure Pursuit" de la librería de Navigation Toolbox. 
El vídeo del funcionamiento de este modelo preparado para que el piero salga de clase (sin evitar obstáculos), es el siguiente:

[![Video demostratvo Salir de clase sin obstáculos](https://img.youtube.com/vi/2jOPRIdVmmE/0.jpg)](https://www.youtube.com/watch?v=2jOPRIdVmmE)

### Salir de clase con Waypoints y evitando obstaculos

El objetivo de este proyecto es permitir que el robot, denominado PIERO, salga de clase de manera autónoma utilizando consignas de velocidad lineal y angular deseadas. Este proceso se llevará a cabo mediante una función de Matlab que implementa el modelo PurePursuit. Además, se busca incorporar la capacidad de esquivar obstáculos, y para ello se ha elegido utilizar la herramienta State Flow.

Es importante destacar que tanto Matlab como State Flow han sido empleados en modelos anteriores, por lo que se comprende su funcionamiento y modo de operación. La diferencia principal radica en la integración de la información proporcionada por cada herramienta, así como en la implementación específica para evitar obstáculos utilizando State Flow.

El modelo de State Flow seleccionado para la tarea de esquivar obstáculos será crucial en la autonomía de PIERO, permitiéndole tomar decisiones dinámicas en tiempo real para sortear cualquier impedimento que pueda surgir en su camino.

<img src="/fotos/im47.png" alt="waypoints">

En esta imagen se implementa el control 

<img src="/fotos/im48.png" alt="waypoints">
El bloque Sonars, nos da las medidas de los sensores, y según esta, se señalizará con el bloque de señalización. Por último, se realizará el control reactivo mediante el StateFlow.
El otro circuito que se muestra, es el propio modelo de movimiento del Piero, controlado esta vez por el modulo pure pursuit, con la trayectoria predefinida en la matriz.


Al simular el Piero con la trayectoria predefinida obtenemos la siguiente gráfica:
<img src="/fotos/im49.png" alt="waypoints">
Como se puede apreciar en la gráfica, el control implementado es bastante preciso y nos permite realizar trayectorias específicas esquivando obstáculos con gran habilidad. Es por eso que el método implementado en el Piero será este.

[![Video demostratvo Salir de clase con obstáculos](https://img.youtube.com/vi/SUVn5_DVqa4/0.jpg)](https://www.youtube.com/watch?v=SUVn5_DVqa4)

## Parte extra: Implementación de una video cámara y de un mando para controlar la orientación de la dicha cámara y la posición del Piero

Toda la documentación de esta parte del proyecto está desarrollada en detalle en un repositorio a parte, al cual se puede acceder haciendo click en el siguiente enlace: [Ver proyecto](https://github.com/dxvidlf/Camara-para-Robot-Movil)

## Resumen de enlaces a todos los videos


 Piero moviéndose en línea recta con poca desviación en 4.1 metros (calibración encoders)
 
   [https://youtu.be/kBWgP7cRCLU](https://youtu.be/kBWgP7cRCLU)
   
 Modelo de señalización de obstáculos en función de los sensores
 
   [https://youtu.be/zqo0bPWz1SM](https://youtu.be/zqo0bPWz1SM)
   
 Calibración de los motores y zona muerta
 
   [https://youtu.be/Tgh0nENfXSM](https://youtu.be/Tgh0nENfXSM)
   
 Piero evitando paredes (en un rincón y cerca de la puerta)
 
   [https://youtu.be/VB6QkytTeBs](https://youtu.be/VB6QkytTeBs)

 Demostración del control en bucle abierto
 
   [https://youtu.be/OU3vdd7K3cA](https://youtu.be/OU3vdd7K3cA)

 Demostración del control en bucle cerrado
 
   [https://youtu.be/-JRCHIIsw-Q](https://youtu.be/-JRCHIIsw-Q)

 Cinemática
 
   [https://youtu.be/YsWtWxGMNfY](https://youtu.be/YsWtWxGMNfY)

 Calibración del control de orientación:
 
   - 10 Vueltas incompletas:  [https://youtu.be/_kR8HMQxs_Y](https://youtu.be/_kR8HMQxs_Y)
   - 10 vueltas completas:    [https://youtu.be/Sdw2mhJyUDc](https://youtu.be/Sdw2mhJyUDc)

 Saliendo de clase usando Matlab Function
 
  [https://youtu.be/nUrU1EKVeNY](https://youtu.be/nUrU1EKVeNY)

 Saliendo de clase sin que haya ningún obstáculo
 
   [https://youtu.be/2jOPRIdVmmE](https://youtu.be/2jOPRIdVmmE)
   
 Saliendo de clase evitando los obstáculos
 
   [https://youtu.be/SUVn5_DVqa4](https://youtu.be/SUVn5_DVqa4)
   
 Demostración del proyecto con la cámara incorporada
 
   - Piero con cámara rotatoria:  [https://youtu.be/6Vd62COOqGI](https://youtu.be/6Vd62COOqGI)
   - Cámara controlada por mando: [https://youtu.be/HZeDFFgzWQ8](https://youtu.be/HZeDFFgzWQ8)
