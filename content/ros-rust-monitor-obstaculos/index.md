+++
title = "Monitor de obstaculos para ROS con Rust"
date = 2021-07-31
[taxonomies]
categories = ["article"]
tags = ["Rust", "ROS"]
+++

# Monitoreando obstaculos

Esta es una traduccion a Rust de una implementacion hecha en `Cpp` por Justin Huang

aqui les dejo el video:

{{ youtube(id="2Cmdu6mkxD0") }}


Me parecio interesante porque es como un "hola mundo" un poco mas avanzado.
Ademas es una linda oportunidad para probar si podemos generar la misma
ergonomia con Rust.

## Definicion del problema

Tenemos nuestro robot que publica su posicion a traves del topic `/odom` y
tenemos de antemano las posiciones de los obstaculos que estan en el mapa donde
transita el robot, entonces vamos a generar un nodo que monitorea cual es el
obstaculo mas cercano.

## Solucion del problema en Cpp

Para que podamos comparar las implementaciones, voy a poner aca la implementacion
en `Cpp`.

```cpp
#include <vector>
#include <string>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "location_monitor/LandmarkDistance.h"
using location_monitor::LandmarkDistance;

struct Landmark {
   std::string name;
   double x;
   double y;
};

struct LandmarkMonitor {
   std::vector<Landmark> landmarks;

   ros::Publisher landmark_publisher;

   LandmarkMonitor(const ros::Publisher& landmark_pub): landmarks(), landmark_publisher(landmark_pub) {
      init_landmarks();
   }

   double get_distance(Landmark landmark, double x, double y) {
      double diff_x = landmark.x - x;
      double diff_y = landmark.y - y;
      return std::sqrt(diff_x * diff_x + diff_y * diff_y);
   }

   LandmarkDistance find_closest(double x, double y) {
      LandmarkDistance result;
      result.distance = -1;
      for (size_t i = 0; i < landmarks.size(); ++i) {
         const Landmark& landmark = landmarks[i];
         double distance = get_distance(landmark, x, y);
         if(result.distance < 0 || distance < result.distance) {
            result.name = landmark.name;
            result.distance = distance;
         }
      }
      return result;
   }

   void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
      double x = msg->pose.pose.position.x;
      double y = msg->pose.pose.position.y;
      LandmarkDistance landmarkd_distance = find_closest(x, y);
      landmark_publisher.publish(landmarkd_distance);
   }

   void init_landmarks() {
      landmarks.push_back(Landmark{"unit_box", 1.0, 1.0});
      landmarks.push_back(Landmark{"unit_cylinder", 1.0, -1.0});
      landmarks.push_back(Landmark{"unit_sphere", -1.0, 0.0});
   }

};

int main(int argc, char** argv) {
   ros::init(argc, argv, "location_monitor");
   ros::NodeHandle node_handle;
   ros::Publisher landmark_publisher = node_handle.advertise<LandmarkDistance>("closest_landmark", 10);
   LandmarkMonitor monitor(landmark_publisher);
   ros::Subscriber subscriber = node_handle.subscribe("odom", 10, &LandmarkMonitor::OdomCallback, &monitor);
   ros::spin();
   return 0;
}
```

Esta implementacion es mas "sencilla" que la hecha por Huang ya que usa solo `structs`

El robot que utilizo es el `turtlebot3` y el mapa es uno hecho a partir del mapa `empty`
al que le agregue un cilindro, una esfera y un box. En esta implementacion el
utiliza un mensaje custom al que llamamos `LandmarkDistance.msg` que esta definido
como:

```text
string name # este es al nombre que le asignamos a el landmark
float64 distance # esta es la distancia que nos separa del landmark
```

ROS genera el codigo de la definicion de esos types y los pone en un archivo `.h`
al que accedemos con:

`#include "location_monitor/LandmarkDistance.h"`

Hay que tener en cuenta que esto tiene que estar configurado en los archivos de
compilacion `package.xml` y `CMakeLists.txt` correctamente(si esto te parece chino
tenes que ver el tutorial oficial sobre mensajes)

## Implementacion en Rust

la implementacion en Rust es la siguiente:

```text
use env_logger;
use rosrust;
use rosrust::Publisher;
use static_math::V2;
use std::sync::mpsc;
use std::time::Duration;

use rosrust_msg::location_monitor::LandmarkDistance;

#[derive(Debug)]
struct Landmark {
    name: String,
    position: V2<f64>
}

impl Landmark {
    fn new(name: String, position: V2<f64>) -> Self {
        Self{name, position}
    }
}

struct LocationMonitor {
    landmarks: Vec<Landmark>,
    publisher: Publisher<LandmarkDistance>
}

impl LocationMonitor {
    fn init(publisher: Publisher<LandmarkDistance>) -> Self {
        let landmarks = vec![Landmark::new("unit_box".to_string(), V2::new_from(1.0, 1.0)),
                         Landmark::new("unit_cylinder".to_string(), V2::new_from(1.0, -1.0)),
                         Landmark::new("unit_sphere".to_string(), V2::new_from(-1.0, 0.0))];
        Self{landmarks, publisher}
    }

    fn get_distance(landmark: &Landmark, point: V2<f64>) -> f64 {
        let diff = landmark.position - point;
        f64::sqrt(diff[0] * diff[0] + diff[1] * diff[1])
    }

    fn find_closest(&self, point: V2<f64>) -> LandmarkDistance {
        let mut result = LandmarkDistance{name: "".to_string(), distance: -1.0};
        for landmark in &self.landmarks {
            let distance = Self::get_distance(landmark, point);
            if result.distance < 0.0 || distance < result.distance {
                result.name = landmark.name.clone();
                result.distance = distance;
            }
        }
        result
    }
}

fn main() {
    env_logger::init();

    rosrust::init("location_monitor_rust");
    let (tx, rx) = mpsc::channel();
    let landmark_publisher = rosrust::publish("landmark_distance", 2).unwrap();
    let monitor = LocationMonitor::init(landmark_publisher);

    // NOTE(elsuizo:2021-07-21): nos subscribimos al topic "/odom" y pasamos ese mensaje a traves
    // del thread
    let subscriber_info = rosrust::subscribe("odom", 2, move |msg: rosrust_msg::nav_msgs::Odometry| {
        // Callback for handling received messages
        tx.send(msg).unwrap();
    })
    .unwrap();
    while rosrust::is_ok() {
        // NOTE(elsuizo:2021-07-21): esperamos la recepcion del mensaje
        if let Ok(msg) = rx.recv_timeout(Duration::from_millis(10)) {
            let x = msg.pose.pose.position.x;
            let y = msg.pose.pose.position.y;
            let position = V2::new_from(x, y);
            let landmark_distance = monitor.find_closest(position);
            monitor.publisher.send(landmark_distance);
        }
    }

    rosrust::spin();
}
```

Como dijimos nos subscribimos al topic `/odom` con el que obtenemos la posicion
`(x, y)` del robot en el mapa con esta informacion calculamos la distancia con
respecto al conjunto de obstaculos y verificamos cual es el mas cercano. A esa
informacion la publicamos como un mensaje `LandmarkDistance` que definimos en
el codigo `Cpp`. Notemos que la implementacion de Rust genera tambien el codigo
necesario para esas definiciones sin que tengamos que hacer nada, solo la importamos
con la linea de codigo:

`use rosrust_msg::location_monitor::LandmarkDistance;`

Y como todo lo que definimos tiene su type bien establecido podemos incorporarlo
al codigo sin problemas:

```rust
struct LocationMonitor {
    landmarks: Vec<Landmark>,
    publisher: Publisher<LandmarkDistance>
}
```

Una de las particularidades de la implementacion en Rust es que debemos utilizar
threads para compartir informacion y dado que el lenguaje nos garantiza que es
seguro no debemos preocuparnos por cosas que pasan en otros lenguajes como por
ejemplo `data races`

```rust
let subscriber_info = rosrust::subscribe("odom", 2, move |msg: rosrust_msg::nav_msgs::Odometry| {
  // Callback for handling received messages
  tx.send(msg).unwrap();
})
```

Aqui enviamos el `msg` a traves de un thread cada vez que recibimos el mismo en
el topic `/odom`. Como contrapartida hacemos la recepcion de manera segura con

```rust
if let Ok(msg) = rx.recv_timeout(Duration::from_millis(10)) {
   let x = msg.pose.pose.position.x;
   let y = msg.pose.pose.position.y;
   let position = V2::new_from(x, y);
   let landmark_distance = monitor.find_closest(position);
   monitor.publisher.send(landmark_distance);
}
```

donde hacemos el llamado a los metodos que nos permiten calcular y publicar cual
es el obstaculo que esta mas cerca.

Por ultimo podemos ver todo esto en accion en el siguiente video. Saludos!!!


{{ youtube(id="r3a2wXH-0N0") }}
