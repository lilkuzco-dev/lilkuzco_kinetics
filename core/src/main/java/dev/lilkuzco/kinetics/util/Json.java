package dev.lilkuzco.kinetics.util;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Minimal RFC 8259 reader, ~200 lines, no dependencies.
 *
 * <p>{@code kinetics-core} deliberately has zero third-party dependencies: it is consumed by
 * warfront, naval and aircraft as well as cosmos, and a shared physics library that drags a
 * JSON stack behind it becomes a versioning problem for all of them. It also keeps I7 honest
 * - nothing outside this module can change how a profile parses.
 *
 * <p>Objects deserialise to {@link LinkedHashMap} rather than {@code HashMap}. That is a
 * determinism requirement, not a style choice: anything that iterates a parsed object must
 * see keys in document order on every machine and every run (I7).
 */
public final class Json {

    private final String src;
    private int pos;

    private Json(String src) { this.src = src; this.pos = 0; }

    public static Object parse(String text) {
        Json p = new Json(text);
        p.skipWhitespace();
        Object v = p.readValue();
        p.skipWhitespace();
        if (p.pos < p.src.length()) {
            throw p.error("trailing content after the top-level value");
        }
        return v;
    }

    @SuppressWarnings("unchecked")
    public static Map<String, Object> parseObject(String text) {
        Object v = parse(text);
        if (!(v instanceof Map)) {
            throw new JsonException("expected a JSON object at the top level, found "
                    + (v == null ? "null" : v.getClass().getSimpleName()));
        }
        return (Map<String, Object>) v;
    }

    private Object readValue() {
        if (pos >= src.length()) throw error("unexpected end of input");
        char c = src.charAt(pos);
        return switch (c) {
            case '{' -> readObject();
            case '[' -> readArray();
            case '"' -> readString();
            case 't' -> readLiteral("true", Boolean.TRUE);
            case 'f' -> readLiteral("false", Boolean.FALSE);
            case 'n' -> readLiteral("null", null);
            default -> readNumber();
        };
    }

    private Map<String, Object> readObject() {
        Map<String, Object> map = new LinkedHashMap<>();
        expect('{');
        skipWhitespace();
        if (peek() == '}') { pos++; return map; }
        while (true) {
            skipWhitespace();
            if (peek() != '"') throw error("expected a quoted key");
            String key = readString();
            skipWhitespace();
            expect(':');
            skipWhitespace();
            map.put(key, readValue());
            skipWhitespace();
            char c = peek();
            if (c == ',') { pos++; continue; }
            if (c == '}') { pos++; return map; }
            throw error("expected ',' or '}' in object");
        }
    }

    private List<Object> readArray() {
        List<Object> list = new ArrayList<>();
        expect('[');
        skipWhitespace();
        if (peek() == ']') { pos++; return list; }
        while (true) {
            skipWhitespace();
            list.add(readValue());
            skipWhitespace();
            char c = peek();
            if (c == ',') { pos++; continue; }
            if (c == ']') { pos++; return list; }
            throw error("expected ',' or ']' in array");
        }
    }

    private String readString() {
        expect('"');
        StringBuilder sb = new StringBuilder();
        while (true) {
            if (pos >= src.length()) throw error("unterminated string");
            char c = src.charAt(pos++);
            if (c == '"') return sb.toString();
            if (c != '\\') { sb.append(c); continue; }
            if (pos >= src.length()) throw error("unterminated escape");
            char e = src.charAt(pos++);
            switch (e) {
                case '"' -> sb.append('"');
                case '\\' -> sb.append('\\');
                case '/' -> sb.append('/');
                case 'b' -> sb.append('\b');
                case 'f' -> sb.append('\f');
                case 'n' -> sb.append('\n');
                case 'r' -> sb.append('\r');
                case 't' -> sb.append('\t');
                case 'u' -> {
                    if (pos + 4 > src.length()) throw error("truncated \\u escape");
                    sb.append((char) Integer.parseInt(src.substring(pos, pos + 4), 16));
                    pos += 4;
                }
                default -> throw error("unknown escape '\\" + e + "'");
            }
        }
    }

    private Double readNumber() {
        int start = pos;
        if (peek() == '-' || peek() == '+') pos++;
        while (pos < src.length()) {
            char c = src.charAt(pos);
            if ((c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-') {
                pos++;
            } else {
                break;
            }
        }
        String text = src.substring(start, pos);
        if (text.isEmpty()) throw error("expected a number");
        try {
            return Double.valueOf(text);
        } catch (NumberFormatException nfe) {
            throw error("malformed number '" + text + "'");
        }
    }

    private Object readLiteral(String word, Object value) {
        if (!src.startsWith(word, pos)) throw error("expected '" + word + "'");
        pos += word.length();
        return value;
    }

    private void skipWhitespace() {
        while (pos < src.length()) {
            char c = src.charAt(pos);
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') { pos++; } else { break; }
        }
    }

    private char peek() {
        if (pos >= src.length()) throw error("unexpected end of input");
        return src.charAt(pos);
    }

    private void expect(char c) {
        if (pos >= src.length() || src.charAt(pos) != c) {
            throw error("expected '" + c + "'");
        }
        pos++;
    }

    /** Errors carry line and column - profile authors read these (Section 8: human-readable). */
    private JsonException error(String message) {
        int line = 1;
        int col = 1;
        for (int i = 0; i < Math.min(pos, src.length()); i++) {
            if (src.charAt(i) == '\n') { line++; col = 1; } else { col++; }
        }
        return new JsonException("JSON error at line " + line + ", column " + col + ": " + message);
    }

    public static final class JsonException extends RuntimeException {
        public JsonException(String message) { super(message); }
    }
}
