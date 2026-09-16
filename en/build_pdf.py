#!/usr/bin/env python3
"""
Build the translated CH32V407/V467 Application Manual PDF from HTML chapters.
Uses WeasyPrint to convert HTML to PDF with embedded figures.
"""

import os, sys, json, re
from pathlib import Path

# WeasyPrint
try:
    from weasyprint import HTML, CSS
except ImportError:
    print("Installing weasyprint...")
    os.system(f"{sys.executable} -m pip install weasyprint")
    from weasyprint import HTML, CSS

BASE_DIR = Path(__file__).parent
FIGURES_DIR = BASE_DIR / "figures"
OUTPUT_PDF = BASE_DIR / "CH32V407V467_Reference_Manual_EN.pdf"

# CSS for the document - professional technical manual style
CSS_STYLE = """
@page {
    size: A4;
    margin: 2cm 1.8cm 2cm 1.8cm;
    @top-center {
        content: "CH32V407/V467 Application Manual (Translation)";
        font-size: 8pt;
        color: #666;
    }
    @bottom-center {
        content: "Page " counter(page) " of " counter(pages);
        font-size: 8pt;
        color: #666;
    }
    @bottom-right {
        content: "V1.1";
        font-size: 8pt;
        color: #999;
    }
}

@page :first {
    @top-center { content: ""; }
    @bottom-center { content: ""; }
    @bottom-right { content: ""; }
}

body {
    font-family: "Noto Sans", "DejaVu Sans", "Arial", sans-serif;
    font-size: 10pt;
    line-height: 1.5;
    color: #222;
}

h1 {
    font-size: 20pt;
    color: #1a5276;
    border-bottom: 2px solid #1a5276;
    padding-bottom: 6px;
    margin-top: 30px;
    page-break-before: always;
}
h1:first-of-type { page-break-before: avoid; }

h2 {
    font-size: 14pt;
    color: #2471a3;
    margin-top: 20px;
    border-bottom: 1px solid #ccc;
    padding-bottom: 3px;
}

h3 {
    font-size: 12pt;
    color: #2e86c1;
    margin-top: 15px;
}

h4 {
    font-size: 11pt;
    color: #3498db;
    margin-top: 12px;
}

p {
    text-align: justify;
    margin: 6px 0;
}

/* Register bit-field table */
table.reg-bits {
    width: 100%;
    border-collapse: collapse;
    font-size: 8pt;
    margin: 10px 0;
}
table.reg-bits td {
    border: 1px solid #888;
    text-align: center;
    padding: 2px 1px;
    vertical-align: middle;
}
table.reg-bits td.reserved {
    background-color: #f0f0f0;
    color: #999;
}

/* Register field description table */
table.reg-fields {
    width: 100%;
    border-collapse: collapse;
    font-size: 9pt;
    margin: 8px 0;
}
table.reg-fields th {
    background-color: #2471a3;
    color: white;
    padding: 4px 6px;
    text-align: left;
    border: 1px solid #1a5276;
}
table.reg-fields td {
    border: 1px solid #bbb;
    padding: 3px 6px;
    vertical-align: top;
}
table.reg-fields tr:nth-child(even) {
    background-color: #f8f9fa;
}

/* Generic data table */
table.data-table {
    width: 100%;
    border-collapse: collapse;
    font-size: 9pt;
    margin: 10px 0;
}
table.data-table caption {
    font-weight: bold;
    font-size: 10pt;
    margin-bottom: 5px;
    color: #1a5276;
}
table.data-table th {
    background-color: #2471a3;
    color: white;
    padding: 4px 6px;
    text-align: left;
    border: 1px solid #1a5276;
}
table.data-table td {
    border: 1px solid #bbb;
    padding: 3px 6px;
    vertical-align: top;
}
table.data-table tr:nth-child(even) {
    background-color: #f8f9fa;
}

/* Figure */
figure {
    text-align: center;
    margin: 15px 0;
    page-break-inside: avoid;
}
figure img {
    max-width: 100%;
    max-height: 650px;
    border: 1px solid #ddd;
}
figure figcaption {
    font-size: 9pt;
    color: #555;
    margin-top: 5px;
    font-weight: bold;
}

/* Code block */
pre, code {
    font-family: "DejaVu Sans Mono", "Courier New", monospace;
    font-size: 8.5pt;
}
pre {
    background-color: #f5f5f5;
    border: 1px solid #ddd;
    padding: 8px;
    overflow-x: auto;
    white-space: pre-wrap;
    word-wrap: break-word;
}

/* Note */
.note {
    background-color: #fff3cd;
    border-left: 3px solid #ffc107;
    padding: 6px 10px;
    margin: 8px 0;
    font-size: 9pt;
}

/* Register list table */
table.reg-list {
    width: 100%;
    border-collapse: collapse;
    font-size: 9pt;
    margin: 10px 0;
}
table.reg-list th {
    background-color: #2471a3;
    color: white;
    padding: 4px 6px;
    border: 1px solid #1a5276;
}
table.reg-list td {
    border: 1px solid #bbb;
    padding: 3px 6px;
}

/* Table of contents */
.toc h1 {
    page-break-before: avoid;
}
.toc-entry {
    margin: 3px 0;
    font-size: 10pt;
}
.toc-entry.chapter {
    font-weight: bold;
    color: #1a5276;
    margin-top: 8px;
}
.toc-entry.section {
    margin-left: 20px;
    color: #333;
}

/* Cover page */
.cover {
    text-align: center;
    page-break-after: always;
    padding-top: 120px;
}
.cover h1 {
    font-size: 28pt;
    color: #1a5276;
    border: none;
    page-break-before: avoid;
}
.cover .subtitle {
    font-size: 16pt;
    color: #555;
    margin-top: 10px;
}
.cover .info {
    margin-top: 60px;
    font-size: 12pt;
    color: #777;
}
.cover .note {
    margin-top: 80px;
    font-size: 9pt;
    color: #999;
    text-align: center;
    background: none;
    border: none;
    padding: 0;
}
"""


def build_cover_html():
    """Generate cover page HTML."""
    return """
<div class="cover">
    <h1>CH32V407 / V467</h1>
    <div class="subtitle">Application Manual</div>
    <div class="info">
        <p>Original: CH32V407、V467 应用手册 V1.1</p>
        <p>Translation: English Technical Translation</p>
        <p>Source: WCH (wch.cn)</p>
    </div>
    <div class="note">
        <p>This is an unofficial technical translation for personal use only.</p>
        <p>Original Chinese document © WCH. All technical content belongs to the original authors.</p>
        <p>Figures are extracted from the original PDF and retain Chinese labels.</p>
    </div>
</div>
"""


def build_toc_html(chapter_titles):
    """Generate table of contents HTML."""
    html = '<div class="toc">\n'
    html += '<h1>Table of Contents</h1>\n'
    for ch_num, ch_title in chapter_titles:
        html += f'<div class="toc-entry chapter">Chapter {ch_num}: {ch_title}</div>\n'
    html += '</div>\n'
    return html


def load_chapter_html(ch_num):
    """Load a chapter's HTML file if it exists."""
    ch_file = BASE_DIR / f"ch{ch_num:02d}.html"
    if ch_file.exists():
        return ch_file.read_text(encoding='utf-8')
    return None


def build_pdf():
    """Build the complete PDF from all chapter HTML files."""
    
    # Chapter titles (Chinese -> English)
    chapter_titles = [
        (1, "Memory and Bus Architecture"),
        (2, "Power Control (PWR)"),
        (3, "Reset and Clock Control (RCC)"),
        (4, "Backup Registers (BKP)"),
        (5, "Cyclic Redundancy Check (CRC)"),
        (6, "Real-Time Clock (RTC)"),
        (7, "Independent Watchdog (IWDG)"),
        (8, "Window Watchdog (WWDG)"),
        (9, "Central Processing Unit (CPU)"),
        (10, "GPIO and Alternate Functions (GPIO/AFIO)"),
        (11, "Direct Memory Access Control (DMA)"),
        (12, "Analog-to-Digital Conversion (ADC)"),
        (13, "Digital-to-Analog Conversion (DAC)"),
        (14, "Advanced Timer (ADTM)"),
        (15, "General Purpose Timer (GPTM)"),
        (16, "Basic Timer (BCTM)"),
        (17, "Universal Synchronous/Asynchronous Receiver Transmitter (USART)"),
        (18, "Two-Wire Communication Bus (I2C)"),
        (19, "I3C Bus (I3C)"),
        (20, "Serial Peripheral Interface (SPI/I2S)"),
        (21, "Operational Amplifier (OPA)"),
        (22, "USB High-Speed Host/Device Controller (USBHS)"),
        (23, "Digital Video Port (DVP)"),
        (24, "SDIO Interface (SDIO)"),
        (25, "Controller Area Network (CAN)"),
        (26, "Flexible Static Memory Controller (FSMC)"),
        (27, "LCD-TFT Display Controller (LTDC)"),
        (28, "Addressable RGB (ARGB)"),
        (29, "Ethernet Controller (ETH)"),
        (30, "Random Number Generator (RNG)"),
        (31, "Electronic Signature (ESIG)"),
        (32, "Flash and User Option Bytes (FLASH)"),
        (33, "Extended Configuration (EXTEN)"),
        (34, "Pseudo-Static Random Access Memory (PSRAM)"),
        (35, "Debug Support (DBG)"),
    ]
    
    # Build complete HTML
    html_parts = []
    html_parts.append(f'<html><head><meta charset="utf-8"></head><body>')
    html_parts.append(build_cover_html())
    html_parts.append(build_toc_html(chapter_titles))
    
    # Include front matter
    front_matter_file = BASE_DIR / "front_matter.html"
    if front_matter_file.exists():
        html_parts.append(front_matter_file.read_text(encoding='utf-8'))
    
    chapters_found = 0
    for ch_num, ch_title in chapter_titles:
        ch_html = load_chapter_html(ch_num)
        if ch_html:
            html_parts.append(ch_html)
            chapters_found += 1
            print(f"  Chapter {ch_num:2d}: {ch_title} ✓")
        else:
            html_parts.append(f'<h1>Chapter {ch_num}: {ch_title}</h1>')
            html_parts.append(f'<p><em>[Translation in progress]</em></p>')
            print(f"  Chapter {ch_num:2d}: {ch_title} [PENDING]")
    
    html_parts.append('</body></html>')
    
    full_html = '\n'.join(html_parts)
    
    if chapters_found == 0:
        print("\nNo chapter HTML files found. Creating placeholder PDF.")
    
    # Write full HTML for debugging
    debug_html = BASE_DIR / "full_manual.html"
    debug_html.write_text(full_html, encoding='utf-8')
    print(f"\nFull HTML: {debug_html}")
    
    # Convert to PDF
    print(f"\nGenerating PDF: {OUTPUT_PDF}")
    doc = HTML(string=full_html, base_url=str(BASE_DIR))
    doc.write_pdf(str(OUTPUT_PDF), stylesheets=[CSS(string=CSS_STYLE)])
    
    size_mb = OUTPUT_PDF.stat().st_size / (1024*1024)
    print(f"PDF generated: {OUTPUT_PDF} ({size_mb:.1f} MB)")
    print(f"Chapters included: {chapters_found}/{len(chapter_titles)}")


if __name__ == '__main__':
    build_pdf()