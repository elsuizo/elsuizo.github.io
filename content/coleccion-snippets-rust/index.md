+++
title = "Coleccion de snippets cools de Rust"
date = 2021-07-27
[taxonomies]
categories = ["article"]
tags = ["Rust"]
+++

# Los mejores snippets de Rust

Los siguientes snippets de codigo los fui recolectando a lo largo del poco tiempo
que llevo en el lenguaje. Su orden de aparicion no significa nada en ninguna
metrica, esto es solo exposicion de arte y elegancia.

## Snippets con `matches!`

El famoso macro `matches!` en accion:

[`matches doc`](https://doc.rust-lang.org/std/macro.matches.html)

```txt
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

## Snippets con `mathc`

El famoso `match` de Rust. Es una de las construcciones mas potentes del lenguaje
en mi humilde opinion. Lo podriamos catalogar como un `switch` pero mucho mas
poderoso.

El primer ejemplo es una implementacion del `trait` `TryFrom` para una `struct`

```txt
struct Color {
   red: u8,
   green: u8,
   blue: u8,
}
```

Entonces con el siguiente codigo implementamos `TryFrom` para una `tupla` `(i16, i16, i16)`

La magia de esto es que en el lado del `pattern` que es siempre a la izquierda
de cada rama del `mathc` tenemos un alias el cual verifica si el valor esta
en un cierto rango (en este caso queremos que el valor entre en un `u8` por eso
el rango es `0..=255`)

```txt
impl TryFrom<(i16, i16, i16)> for Color {
    type Error = Box<dyn error::Error>;
    fn try_from(tuple: (i16, i16, i16)) -> Result<Self, Self::Error> {
        match tuple {
            (r @ 0..=255, g @ 0..=255, b @ 0..=255) => Ok(Color {
                red: r as u8,
                green: g as u8,
                blue: b as u8,
            }),
            _ => Err("rgb must be 0~255".into()),
        }
    }
}
```

Podemos utilizar un match en la definicion de una variable ya que en el lenguaje
podemos usar cualquier expresion para ello. En el siguiente ejemplo usamos un `match`
para asignar los valores de cada color en una funcion que calcula el falso color
a una imagen:

```txt
/// False color of an image
fn false_color(pixel: image::Rgb<u8>) -> image::Rgb<u8> {
    let m = 255.0 / 43.0;

    let r = match pixel[0] {
        0..=43            => 255.0,
        value @ 44..=86   => -m * (value as f32 - 86.0),
        87..=172          => 0.0,
        value @ 173..=215 => m * (value as f32 - 173.0),
        216..=MAX         => 255.0,
    };
    let g = match pixel[1] {
        value @ 0..=43    => m * value as f32,
        44..=128          => 255.0,
        value @ 129..=172 => -m * (value as f32 - 172.0),
        173..=MAX         => 0.0,
    };
    let b = match pixel[2] {
        0..=86            => 0.0,
        value @ 87..=129  => m * (value as f32 - 87.0),
        130..=213         => 255.0,
        value @ 214..=MAX => -m * (value as f32 - 255.0),
    };

    image::Rgb([r as u8, g as u8, b as u8])
}
```
