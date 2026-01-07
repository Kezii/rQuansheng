//! Lightweight proc-macros for BK4819 register definitions.
//!
//! We intentionally avoid `syn`/`quote` to keep this crate dependency-free and
//! friendly to offline builds.

use proc_macro::{Delimiter, Group, Ident, Literal, TokenStream, TokenTree};

/// Attach a BK4819 register address to a `struct` definition.
///
/// Usage:
/// ```ignore
/// #[address(0x00)]
/// pub struct Reg00 { .. }
/// ```
///
/// Expands to:
/// - `impl Reg00 { pub const ADDRESS: u8 = 0x00; pub const fn get_address() -> u8 { 0x00 } }`
/// - `impl crate::bk4819_n::RegisterAddress for Reg00 { const ADDRESS: u8 = 0x00; }`
#[proc_macro_attribute]
pub fn address(attr: TokenStream, item: TokenStream) -> TokenStream {
    let addr_literal = parse_single_literal(attr);
    let struct_name = find_struct_name(item.clone());

    let extra: TokenStream = format!(
        r#"
impl {name} {{
    pub const ADDRESS: u8 = {addr};
    #[inline]
    pub const fn get_address() -> u8 {{ {addr} }}
}}

impl crate::bk4819_n::RegisterAddress for {name} {{
    const ADDRESS: u8 = {addr};
}}
"#,
        name = struct_name,
        addr = addr_literal
    )
    .parse()
    .expect("generated impl must parse");

    let mut out = TokenStream::new();
    out.extend(item);
    out.extend(extra);
    out
}

fn parse_single_literal(attr: TokenStream) -> String {
    let mut it = attr.into_iter();

    // Accept either:
    // - `0x00`
    // - `(0x00)` (some editors format it like this)
    let first = it.next().expect("expected #[address(<literal>)]");
    let lit = match first {
        TokenTree::Literal(l) => l,
        TokenTree::Group(g) if g.delimiter() == Delimiter::Parenthesis => {
            let mut inner = g.stream().into_iter();
            match inner.next() {
                Some(TokenTree::Literal(l)) => l,
                _ => panic!("expected #[address(<literal>)]"),
            }
        }
        _ => panic!("expected #[address(<literal>)]"),
    };

    // Ensure no extra tokens.
    if it.next().is_some() {
        panic!("expected a single literal in #[address(..)]");
    }

    normalize_u8_literal(lit)
}

fn normalize_u8_literal(lit: Literal) -> String {
    // We keep the literal text (so `0x00` stays hex), but validate it fits in u8.
    let s = lit.to_string();
    let s_trim = s.trim();

    let (radix, digits) = if let Some(rest) = s_trim.strip_prefix("0x") {
        (16u32, rest)
    } else if let Some(rest) = s_trim.strip_prefix("0X") {
        (16u32, rest)
    } else {
        (10u32, s_trim)
    };

    let digits = digits.replace('_', "");
    let value = u32::from_str_radix(&digits, radix).expect("invalid integer literal in #[address]");
    if value > u8::MAX as u32 {
        panic!("#[address] value must fit in u8 (0..=255)");
    }

    s
}

fn find_struct_name(item: TokenStream) -> String {
    let mut seen_struct = false;
    for tt in item {
        match tt {
            TokenTree::Ident(id) if id.to_string() == "struct" => {
                seen_struct = true;
            }
            TokenTree::Ident(id) if seen_struct => {
                // This is the identifier after `struct`.
                return id.to_string();
            }
            TokenTree::Group(Group { .. }) => {
                // Ignore groups; struct name should be in the outer stream.
            }
            _ => {}
        }
    }
    panic!("#[address] can only be used on a `struct` item");
}
