+++
title = "Primeros pasos con Rust"
date = 2021-07-24
[taxonomies]
categories = ["article"]
tags = ["Rust"]
+++


### Presentacion de Rust

Rust es un lenguaje de programacion particular que viene a solucionar un
problema que existe desde hace mucho tiempo con el manejo de memoria seguro
y eficiente.

 - Es seguro: Porque el compilador y el diseño del lenguaje no permite que el programador
   cometa los errores clasicos de manejo de memoria(punteros colgados, dobles frees,
   buffer overflow...etc)

 - ES eficiente: Porque para lograr lo anterior no utiliza ningun "runtime"(como Java)
   o GC("Garbage collector") por lo que hace que el codigo generado sea super
   eficiente(poco consumo de memoria, excelente generacion de codigo maquina)

Al ser un lenguaje relativamente nuevo toma las preferencias y lecciones aprendidas
por la industria del software en cuanto a diseño, por ejemplo esta mas orientado
a los datos ("data oriented") que a la programacion con objetos. Tambien su diseño
se vio muy influenciado por lenguajes funcionales, teniendo un gran soporte para
ese paradigma de programacion.

Como comenzar???. Bueno primero necesitamos instalar el compilador y el _package mannager_
que se llama `cargo`.

Vamos a la pagina del lenguaje:

[`pagina oficial de Rust`](https://www.rust-lang.org/)

vamos a la la pestaña "get started" ---> "installing Rust" y ahi seguimos los
pasos(segun el Sistema operativo que tengamos)

Si todo salio bien debemos tener disponible los siguientes comandos en nuestro
sistema:

 - `rustc`
 - `cargo`

Ahora hagamos nuestro primer programa en Rust. Obviamente y para seguir la tradicion
el famoso hola mundo!!!.

Primero creamos el proyecto con la herramienta `cargo` haciendo en la carpeta
donde queremos tener el mismo:

`cargo new hola-rust`

Si todo fue bien obtenemos el mensaje:

`Created binary (application) `hola-rust` package`

en la misma tenemos los siguientes archivos:

```text
.
├── Cargo.toml
└── src
    └── main.rs

```

Donde:
 - `Cargo.toml`: Es el archivo de configuracion del _package mannager_ donde podemos
   manejar las dependencias de nuestro proyecto, banderas para la compilacion, ...etc
 - `main.rs`: En este archivo vamos a escribir nuestro hola mundo, como ya viene
   casi listo solo tenemos que modificar la `String` (en realidad es un `&str`)
   que esta entre "" y poner el mensaje en nuestro idioma(aunque no estoy de acuerdo
   de programar en español...solo por esta vez)

```rust
fn main() {
    println!("Hola Mundo!!!");
}
```

Y para correr el codigo solo tenemos que hacer:

```bash
cargo run
```

Quien dijo que Rust era dificil ???. Saludos
