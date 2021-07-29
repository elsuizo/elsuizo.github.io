+++
title = "Coleccion de snippets cools de Rust"
date = 2021-07-27
[taxonomies]
categories = ["article"]
tags = ["Rust"]
+++

# Los mejores snippets de Rust comentados

Los siguientes snippets de codigo los fui recolectando a lo largo del poco tiempo
que llevo en el lenguaje. Su orden de aparicion no significa nada en ninguna
metrica, esto es solo exposicion de arte y elegancia.

## Snippet 1

El famoso macro `matches!` en accion:

```rust
enum Foo {
    Bar(u32),
    Baz
}

fn is_bar(foo: Foo) -> bool {
    matches!(foo, Foo::Bar(_))
}

fn is_bar_greather(foo: Foo, thresold: u32) -> bool {
    matches!(foo, Foo::Bar(x) if x > thresold)
}

fn main() {
    let m = Foo::Bar(0);
    let m2 = Foo::Bar(10);
    let m3 = Foo::Baz;
    println!("is: {:}", is_bar(m));
    println!("is: {:}", is_bar(m3));
    println!("is: {:}", is_bar_greather(m2, 1));
}
```
