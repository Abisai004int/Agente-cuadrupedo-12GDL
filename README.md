Codigos utilizados para el funcionamiento del Agente Cuadrupedo 12GDL con vision artificial

Explicacion codigo ESP32-C6 Supermini:
1. Firmware del ESP32-C6 — Control, locomoción y navegación autónoma
El programa desarrollado para el ESP32-C6 constituye el sistema principal de control del robot cuadrúpedo de 12 grados de libertad. Su función es coordinar el movimiento de las cuatro extremidades, interpretar las mediciones de los sensores de distancia y ejecutar la lógica de navegación autónoma. También administra la comunicación con la Raspberry Pi mediante UART y proporciona una interfaz web para consultar el estado del sistema.
Control de actuadores
El robot cuenta con doce servomotores MG996R, distribuidos en cuatro extremidades con tres articulaciones cada una: coxa, fémur y tibia. Para controlar los servomotores se utiliza un módulo PCA9685, conectado al ESP32-C6 mediante el protocolo de comunicación I²C.

El programa configura el controlador PWM a una frecuencia de 50 Hz y establece un intervalo de pulsos de 150 a 600, correspondiente al rango de movimiento configurado para los servomotores. Mediante una función de conversión, los ángulos calculados para cada articulación se transforman en señales PWM que permiten posicionar los actuadores.
Cinemática inversa y generación de movimiento
Para calcular la posición angular de las articulaciones se implementa un algoritmo de cinemática inversa. Este recibe las coordenadas espaciales \(X\), \(Y\) y \(Z\) deseadas para el extremo de cada extremidad y calcula los ángulos necesarios de la coxa, el fémur y la tibia.

El cálculo considera las dimensiones geométricas de los segmentos de las patas, así como los desplazamientos angulares necesarios para compensar la posición mecánica de los servomotores. También incorpora límites articulares y una comprobación del espacio de trabajo para evitar posiciones que no puedan alcanzarse físicamente.

A partir de los ángulos obtenidos, el programa genera trayectorias cíclicas para las cuatro extremidades. Cada ciclo de marcha se divide en dos fases:
- Fase de balanceo: la extremidad se levanta y desplaza hacia adelante para preparar el siguiente apoyo.
- Fase de apoyo: la extremidad desciende y se desplaza hacia atrás respecto al cuerpo, produciendo el avance del robot.
Las cuatro patas utilizan desfases temporales para coordinar sus movimientos y mantener una secuencia de marcha. El mismo sistema de generación de trayectorias se utiliza para realizar giros controlados.
Detección de obstáculos
Para identificar obstáculos se emplean tres sensores de distancia VL53L0X, distribuidos en la parte frontal, izquierda y derecha del robot. Estos sensores se comunican con el ESP32-C6 mediante I²C.

Debido a que los tres sensores utilizan inicialmente la misma dirección I²C, el programa controla individualmente sus terminales XSHUT para inicializarlos de manera secuencial y asignarles direcciones diferentes: 0x30, 0x31 y 0x32.

Las mediciones se realizan de forma periódica y se utilizan dos umbrales de detección: 300 mm para la zona frontal y 500 mm para las zonas laterales. Para reducir falsas detecciones, el sistema requiere tres lecturas consecutivas que confirmen la presencia de un obstáculo antes de modificar el comportamiento del robot.
Lógica de navegación autónoma
La navegación se implementa mediante una máquina de estados que determina el comportamiento del robot dependiendo de las condiciones detectadas por los sensores y de las respuestas obtenidas de la Raspberry Pi.

Cuando el robot avanza y detecta un obstáculo frontal, detiene su locomoción y solicita a la Raspberry Pi realizar una búsqueda visual. Si la Raspberry Pi identifica el objeto objetivo, devuelve el mensaje ENCONTRADA y el ESP32-C6 detiene el robot, marcando la misión como completada.

Si el objeto no es identificado, la Raspberry Pi envía NO_ENCONTRADA. En ese caso, el ESP32-C6 evalúa las distancias laterales y determina qué dirección inspeccionar o tomar como ruta. Si ambos lados presentan espacio disponible, alterna la dirección preferida para evitar elegir siempre el mismo sentido. Cuando no existe una ruta lateral disponible, ejecuta un giro de 180 grados.

Después de realizar un giro, el sistema comprueba nuevamente la distancia frontal y reanuda la marcha cuando confirma que el camino está libre. De esta manera, la navegación combina información de proximidad con reconocimiento visual para tomar decisiones de movimiento.
Comunicación con la Raspberry Pi
La comunicación entre el ESP32-C6 y la Raspberry Pi 5 se realiza mediante UART, utilizando una velocidad de transmisión de 115200 baudios. El ESP32-C6 envía el mensaje BUSCAR_MANZANA cuando necesita que la computadora realice una búsqueda visual.

La Raspberry Pi procesa la solicitud y responde con alguno de los siguientes mensajes:
- ENCONTRADA: el objeto objetivo fue identificado.
- NO_ENCONTRADA: el objeto no fue identificado durante la búsqueda.
- ERROR_VISION: ocurrió un error durante el proceso de reconocimiento.
El ESP32-C6 interpreta estas respuestas y modifica el estado de navegación según corresponda. Esta separación de responsabilidades permite que el microcontrolador gestione el movimiento y los sensores, mientras que la Raspberry Pi se encarga del procesamiento de imágenes y la inteligencia artificial.
Monitoreo y control mediante Wi-Fi
El firmware incorpora un servidor web que permite supervisar el funcionamiento del robot desde un navegador. Para ello, el ESP32-C6 se conecta a la red Wi-Fi generada por la Raspberry Pi y utiliza una dirección IP fija.

La interfaz web muestra información como el estado de conexión, la dirección IP, el tiempo de funcionamiento, las distancias medidas por los tres sensores, el estado actual de navegación y los últimos mensajes intercambiados mediante UART. También conserva un historial de comunicación para facilitar la supervisión y depuración del sistema.

Además, la interfaz incluye dos acciones de control: iniciar la búsqueda autónoma y reiniciar la navegación. La información se actualiza periódicamente mediante solicitudes HTTP, mientras que los comandos de control se envían al servidor mediante solicitudes POST.
Funcionamiento general del programa
Durante el arranque, el ESP32-C6 inicializa los periféricos de comunicación, el controlador PWM, los tres sensores de distancia y la calibración de las articulaciones. Posteriormente, establece la conexión Wi-Fi con la Raspberry Pi, inicia el servidor web y comienza la marcha autónoma.

Durante la ejecución, el programa mantiene diferentes tareas de manera periódica: actualiza la locomoción, consulta los sensores, procesa los mensajes UART y atiende las solicitudes del servidor web. Esta organización permite integrar el movimiento, la percepción del entorno y la supervisión del robot dentro de un mismo firmware.
