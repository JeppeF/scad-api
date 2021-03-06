## Functions

<dl>
<dt><a href="#css2rgb">css2rgb(String)</a> ⇒</dt>
<dd><p>Converts an CSS color name to RGB color.</p>
</dd>
<dt><a href="#rgb2hsl">rgb2hsl(Number, Number, Number)</a> ⇒</dt>
<dd><p>Converts an RGB color value to HSL. Conversion formula
adapted from <a href="http://en.wikipedia.org/wiki/HSL_color_space">http://en.wikipedia.org/wiki/HSL_color_space</a>.
Assumes r, g, and b are contained in the set [0, 1] and
returns h, s, and l in the set [0, 1].</p>
</dd>
<dt><a href="#hsl2rgb">hsl2rgb(Number, Number, Number)</a> ⇒</dt>
<dd><p>Converts an HSL color value to RGB. Conversion formula
adapted from <a href="http://en.wikipedia.org/wiki/HSL_color_space">http://en.wikipedia.org/wiki/HSL_color_space</a>.
Assumes h, s, and l are contained in the set [0, 1] and
returns r, g, and b in the set [0, 1].</p>
</dd>
<dt><a href="#rgb2hsv">rgb2hsv(Number, Number, Number)</a> ⇒</dt>
<dd><p>Converts an RGB color value to HSV. Conversion formula
adapted from <a href="http://en.wikipedia.org/wiki/HSV_color_space">http://en.wikipedia.org/wiki/HSV_color_space</a>.
Assumes r, g, and b are contained in the set [0, 1] and
returns h, s, and v in the set [0, 1].</p>
</dd>
<dt><a href="#hsv2rgb">hsv2rgb(Number, Number, Number)</a> ⇒</dt>
<dd><p>Converts an HSV color value to RGB. Conversion formula
adapted from <a href="http://en.wikipedia.org/wiki/HSV_color_space">http://en.wikipedia.org/wiki/HSV_color_space</a>.
Assumes h, s, and v are contained in the set [0, 1] and
returns r, g, and b in the set [0, 1].</p>
</dd>
<dt><a href="#html2rgb">html2rgb()</a></dt>
<dd><p>Converts a HTML5 color value (string) to RGB values
See the color input type of HTML5 forms
Conversion formula:</p>
<ul>
<li>split the string; &quot;#RRGGBB&quot; into RGB components</li>
<li>convert the HEX value into RGB values</li>
</ul>
</dd>
<dt><a href="#rgb2html">rgb2html()</a></dt>
<dd><p>Converts RGB color value to HTML5 color value (string)
Conversion forumla:</p>
<ul>
<li>convert R, G, B into HEX strings</li>
<li>return HTML formatted string &quot;#RRGGBB&quot;</li>
</ul>
</dd>
</dl>

<a name="css2rgb"></a>

## css2rgb(String) ⇒
Converts an CSS color name to RGB color.

**Kind**: global function  
**Returns**: Array           The RGB representation, or [0,0,0] default  

| Param | Description |
| --- | --- |
| String | s       The CSS color name |

<a name="rgb2hsl"></a>

## rgb2hsl(Number, Number, Number) ⇒
Converts an RGB color value to HSL. Conversion formula
adapted from http://en.wikipedia.org/wiki/HSL_color_space.
Assumes r, g, and b are contained in the set [0, 1] and
returns h, s, and l in the set [0, 1].

**Kind**: global function  
**Returns**: Array           The HSL representation  

| Param | Description |
| --- | --- |
| Number | r       The red color value |
| Number | g       The green color value |
| Number | b       The blue color value |

<a name="hsl2rgb"></a>

## hsl2rgb(Number, Number, Number) ⇒
Converts an HSL color value to RGB. Conversion formula
adapted from http://en.wikipedia.org/wiki/HSL_color_space.
Assumes h, s, and l are contained in the set [0, 1] and
returns r, g, and b in the set [0, 1].

**Kind**: global function  
**Returns**: Array           The RGB representation  

| Param | Description |
| --- | --- |
| Number | h       The hue |
| Number | s       The saturation |
| Number | l       The lightness |

<a name="rgb2hsv"></a>

## rgb2hsv(Number, Number, Number) ⇒
Converts an RGB color value to HSV. Conversion formula
adapted from http://en.wikipedia.org/wiki/HSV_color_space.
Assumes r, g, and b are contained in the set [0, 1] and
returns h, s, and v in the set [0, 1].

**Kind**: global function  
**Returns**: Array           The HSV representation  

| Param | Description |
| --- | --- |
| Number | r       The red color value |
| Number | g       The green color value |
| Number | b       The blue color value |

<a name="hsv2rgb"></a>

## hsv2rgb(Number, Number, Number) ⇒
Converts an HSV color value to RGB. Conversion formula
adapted from http://en.wikipedia.org/wiki/HSV_color_space.
Assumes h, s, and v are contained in the set [0, 1] and
returns r, g, and b in the set [0, 1].

**Kind**: global function  
**Returns**: Array           The RGB representation  

| Param | Description |
| --- | --- |
| Number | h       The hue |
| Number | s       The saturation |
| Number | v       The value |

<a name="html2rgb"></a>

## html2rgb()
Converts a HTML5 color value (string) to RGB values
See the color input type of HTML5 forms
Conversion formula:
- split the string; "#RRGGBB" into RGB components
- convert the HEX value into RGB values

**Kind**: global function  
<a name="rgb2html"></a>

## rgb2html()
Converts RGB color value to HTML5 color value (string)
Conversion forumla:
- convert R, G, B into HEX strings
- return HTML formatted string "#RRGGBB"

**Kind**: global function  
