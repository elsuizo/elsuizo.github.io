+++
title = "Hola mundo con ROS y Rust"
date = 2021-07-25
[taxonomies]
categories = ["article"]
tags = ["Rust", "ROS"]
+++

# Como empezar con ROS y Rust

Si comenzar con ROS ya tiene algunas dificultades porque como se dice
vulgarmente tienen que estar todos los patitos en fila(`CMakes`, `package.xml`,
dependencias, workpace de catkin, ...etc) entonces comenzar con ROS y Rust aun
mas porque ni bien empezas te preguntas pero como integro mi nodo de Rust(hecho
con `cargo`) a un projecto de catkin(que usa `CMake`)???

Bueno no te preocupes hice una herramienta:

[`ros-project-generator`](https://crates.io/crates/ros-project-generator)


que te genera los nodos de ROS hechos
con Rust(hecho con `cargo`) y que se comporta como un nodo "normal" de otro
lenguaje como `python` o `Cpp`, osea genera los nodos y todos los archivos que
se necesitan para que compilen con `CMake`

La herramienta se instala como un _subcommand_ de `cargo` haciendo simplemente:

`cargo install ros-project-generator`

Una vez instalado solo necesitamos un archivo de configuracion `config.yaml` en
nuestro _workspace_ de `catkin` como este:

```yaml
# define a name
project_name: "navigation-robot"

# define a version
version: 0.0.1

# define the packages names
nodes:
   - listener
   - talker
```

Donde se definen el nombre del conjunto de nodos(`project_name`), la version y
la lista de nodos que queremos crear. Luego solo hacemos(siempre en la carpeta
donde esta el archivo de configuracion):

`cargo ros-project-generator`

Si todo salio bien vamos a obtener un mensaje como:

```txt
navigation-robot project generated!!!
```

Ahora que tenemos generados los nodos, deberian aparecer dentro de la carpeta
`src` en este caso como generamos dos nodos nos deberian aparecer `listener` y
`talker`

Ahora para completar este `hola mundo` de ROS deberiamos hacer que el nodo `talker`
publique "Hola mundo" en un topic que le asignemos por ejemplo `"chatter"`

Para ello necesitamos las siguientes lineas de codigo:

```txt
use env_logger;
use rosrust;

fn main() {
    env_logger::init();

    // Initialize node
    rosrust::init("talker");

    // Create publisher
    let chatter_pub = rosrust::publish("chatter", 2).unwrap();

    let log_names = rosrust::param("~log_names").unwrap().get().unwrap_or(false);

    let mut count = 0;

    // Create object that maintains 10Hz between sleep requests
    let rate = rosrust::rate(10.0);

    // Breaks when a shutdown signal is sent
    while rosrust::is_ok() {
        // Create string message
        let mut msg = rosrust_msg::std_msgs::String::default();
        msg.data = format!("hello world from rosrust {}", count);

        // Log event
        rosrust::ros_info!("Publishing: {}", msg.data);

        // Send string message to topic via publisher
        chatter_pub.send(msg).unwrap();

        if log_names {
            rosrust::ros_info!("Subscriber names: {:?}", chatter_pub.subscriber_names());
        }

        // Sleep to maintain 10Hz rate
        rate.sleep();

        count += 1;
    }
}
```

Luego para el nodo `listener` debemos hacer que el mismo se subscriba al topic
que esta publicando el nodo anterior y asi completar este "Hola mundo"

```rust
use env_logger;
use rosrust;

fn main() {
    env_logger::init();

    // Initialize node
    rosrust::init("listener");

    // Create subscriber
    // The subscriber is stopped when the returned object is destroyed
    let subscriber_info = rosrust::subscribe("chatter", 2, |v: rosrust_msg::std_msgs::String| {
        // Callback for handling received messages
        rosrust::ros_info!("Received: {}", v.data);
    })
    .unwrap();

    let log_names = rosrust::param("~log_names").unwrap().get().unwrap_or(false);

    if log_names {
        let rate = rosrust::rate(1.0);
        while rosrust::is_ok() {
            rosrust::ros_info!("Publisher uris: {:?}", subscriber_info.publisher_uris());
            rate.sleep();
        }
    } else {
        // Block the thread until a shutdown signal is received
        rosrust::spin();
    }
}
```

Si todo salio bien vamos a poder compilar nuestros nodos con el comando de siempre
que es(asumiendo que estamos usando `catkin`)

```bash
catkin_make
```

Si todo ha compilado sin errores tenemos ahora entonces los ejecutables de cada
uno de los nodos disponibles para que podamos ejecutarlos, osea que podemos hacer

```bash
rosrun listener listener
```

y en otra terminal el otro nodo:

```bash
rosrun talker talker
```

Para obtener nuestro preciado `Hello World!!!`


{{gif(url="./hola_mundo.png")}}
