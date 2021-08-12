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

## Snippets con `match`

El famoso `match` de Rust. Es una de las construcciones mas potentes del lenguaje
en mi humilde opinion. Lo podriamos catalogar como un `switch` pero mucho mas
poderoso.

El primer ejemplo es una implementacion del `trait` `TryFrom` para una `struct`

```rust
struct Color {
   red: u8,
   green: u8,
   blue: u8,
}
```

Entonces con el siguiente codigo implementamos `TryFrom` para una `tupla` `(i16, i16, i16)`

La magia de esto es que en el lado del `pattern` que es siempre a la izquierda
de cada rama del `match` tenemos un alias el cual verifica si el valor esta
en un cierto rango (en este caso queremos que el valor entre en un `u8` por eso
el rango es `0..=255`)

```rust
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

Podemos utilizar un `match` en la definicion de una variable ya que en el lenguaje
podemos usar cualquier expresion para ello. En el siguiente ejemplo usamos un `match`
para asignar los valores de cada color en una funcion que calcula el falso color
a una imagen:

```rust
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

## Snippets de notacion

En Rust podemos desestructurar casi cualquier expresion para nuestra conveniencia
por ejemplo queremos hacer una funcion que tome un array de tres elementos
y que sume el primer y el ultimo elemento:

```rust
fn add_first_last([x, _, y]: [i32; 3]) -> i32 {
    x + y
}
```

Supongamos que tenemos la siguiente `struct Foo`, tenemos un array de `Foo`s y
queremos iterar solo sobre los elementos `a` de cada `Foo`

```rust
struct Foo {
    a: u32,
    b: &'static str,
    c: &'static str
}

// constructor de Foo
impl Foo {
    fn new(a: u32, b: &'static str, c: &'static str) -> Self {
        Self {a, b, c}
    }
}

fn main() {
    let data = [Foo::new(3, "one", "two"), Foo::new(7, "two", "tree"), Foo::new(37, "l", "pi")];

    for Foo{a, ..} in data {
        println!("a: {}", a);
    }
}
```

## Snippets con iteradores

Rust utiliza iteradores como elemento para iterar sobre colecciones de
elementos esto hace que por ejemplo el compilador sepa de antemano cuando debe
terminar la iteracion y asi no tener que realizar el chequeo en cada una de las
mismas como si pasa por ejemplo en `C`

`for(int i=0; i < 10; i++)`

Por ejemplo supongamos que queremos implementar el producto punto para dos
vectores o arrays de cualquier `length`

```rust
pub fn dot(slice1: &[f32], slice2: &[f32]) -> f32 {
    slice1.iter().zip(slice2).map(|(&a, &b)| a * b).sum()
}

fn main() {
    let a = [1.0, 1.0, 3.0, 4.0];
    let b = [4.0, 2.0, 1.0, 7.0];
    let result = dot(&a, &b);
    println!("result: {}", result);
}
```

