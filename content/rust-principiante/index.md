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
ese paradigma de programacion. Por ultimo como para entrar en calor les dejo como
se ve el clasico "Hola mundo" en Rust.

```rust
fn main() {
    println!("Hello, world!");
}
```
