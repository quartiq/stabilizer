# Persistent Settings Management Serial Interface

## Description
This crate provides a simple means to load, configure, and store device settings over a serial
(i.e. text-based) interface. It is ideal to be used with serial ports and terminal emulators,
and exposes a simple way to allow users to configure device operation.

## Example
Let's assume that your settings structure looks as follows:
```rust
#[derive(miniconf::Tree, ...)]
struct Settings {
    broker: String,
    id: String,
}
```

A user would be displayed the following terminal interface:
```
 help
AVAILABLE ITEMS:
  get [path]
  set <path> <value>
  store [path]
  clear [path]
  platform <cmd>
  help [ <command> ]

> get
Available settings:
/broker: "test" [default: "mqtt"] [not stored]
/id: "04-91-62-d2-a8-6f" [default] [not stored]
```

## Design
Settings are specified in a [`miniconf::TreeKey`] settings tree and are transferred over the
serial interface using JSON encoding. This means that things like strings must be encased in
qutoes.

## Limitations
Currently, there is a hardcoded limit of 128-bytes on the settings path. This is arbitrary and
can be changed if needed.
