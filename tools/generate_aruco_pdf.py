"""
tools/generate_aruco_pdf.py — Generates a Professional Print-Ready PDF of ArUco Waypoint Markers.
Dictionary: cv2.aruco.DICT_4X4_50
Physical Scale: Exact 10.0 cm (100mm) calibration for 3D metric camera homing.
"""

import os
import cv2
import numpy as np
import tempfile

from reportlab.lib.pagesizes import A4
from reportlab.lib import colors
from reportlab.pdfgen import canvas
from reportlab.lib.units import cm, mm

def generate_marker_image(dict_type, marker_id, pixel_size=1000):
    dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, pixel_size, 1)
    margin = int(pixel_size * 0.12)
    bordered = cv2.copyMakeBorder(marker, margin, margin, margin, margin, cv2.BORDER_CONSTANT, value=255)
    return bordered

def draw_header(c, title, page_num, total_pages=4):
    width, height = A4
    c.saveState()
    # Banner background
    c.setFillColor(colors.HexColor("#4338CA"))
    c.rect(0, height - 1.8 * cm, width, 1.8 * cm, stroke=0, fill=1)
    
    # Title text
    c.setFillColor(colors.white)
    c.setFont("Helvetica-Bold", 12)
    c.drawString(1.2 * cm, height - 1.15 * cm, title)
    
    c.setFont("Helvetica", 9)
    c.drawRightString(width - 1.2 * cm, height - 1.15 * cm, f"Page {page_num} of {total_pages} · DICT_4X4_50")
    
    # Sub-header note
    c.setFillColor(colors.HexColor("#475569"))
    c.setFont("Helvetica-Bold", 8)
    c.drawString(1.2 * cm, height - 2.25 * cm, "PRINTING: Print at 100% / Actual Size (Do NOT select 'Fit to Page'). Physical Marker Size = 10.0 cm.")
    c.restoreState()

def draw_calibration_ruler(c, x, y, length_cm=10.0):
    c.saveState()
    c.setStrokeColor(colors.HexColor("#334155"))
    c.setLineWidth(0.9)
    
    length_pt = length_cm * cm
    c.line(x, y, x + length_pt, y)
    
    # Millimeter / Centimeter ticks
    for i in range(int(length_cm) + 1):
        tick_x = x + i * cm
        c.line(tick_x, y - 3, tick_x, y + 6)
        c.setFont("Helvetica", 6.5)
        c.setFillColor(colors.HexColor("#334155"))
        c.drawCentredString(tick_x, y + 8, f"{i}cm")
        
        if i < int(length_cm):
            c.line(x + (i + 0.5) * cm, y - 2, x + (i + 0.5) * cm, y + 4)
            
    c.setFont("Helvetica-Bold", 7.0)
    c.setFillColor(colors.HexColor("#4F46E5"))
    c.drawCentredString(x + length_pt / 2.0, y - 8.5, f"{int(length_cm * 10)} mm Calibration Standard (Verify with ruler)")
    c.restoreState()

def draw_marker_card(c, img_path, x, y, tag_id, tag_label, description, marker_size_cm=10.0):
    c.saveState()
    box_w = marker_size_cm * cm
    box_h = marker_size_cm * cm
    
    # Outer light card boundary
    card_margin = 0.5 * cm
    card_x = x - card_margin
    card_y = y - 1.8 * cm
    card_w = box_w + 2 * card_margin + 6.2 * cm
    card_h = box_h + card_margin + 2.2 * cm
    
    c.setFillColor(colors.HexColor("#F8FAFC"))
    c.setStrokeColor(colors.HexColor("#CBD5E1"))
    c.setLineWidth(0.8)
    c.roundRect(card_x, card_y, card_w, card_h, 8, stroke=1, fill=1)
    
    # Draw Marker Image
    c.drawImage(img_path, x, y, width=box_w, height=box_h, preserveAspectRatio=True)
    
    # Dashed Cutting Guide
    c.setStrokeColor(colors.HexColor("#94A3B8"))
    c.setLineWidth(0.8)
    c.setDash(3, 3)
    c.rect(x, y, box_w, box_h, stroke=1, fill=0)
    c.setDash([])
    
    # Tag Information Column
    info_x = x + box_w + 0.8 * cm
    
    # Tag Badge
    badge_w = 2.4 * cm
    badge_h = 0.65 * cm
    c.setFillColor(colors.HexColor("#4F46E5"))
    c.roundRect(info_x, y + box_h - badge_h, badge_w, badge_h, 4, stroke=0, fill=1)
    c.setFillColor(colors.white)
    c.setFont("Helvetica-Bold", 9.5)
    c.drawCentredString(info_x + badge_w / 2.0, y + box_h - badge_h + 4.5, f"TAG ID #{tag_id}")
    
    # Title
    c.setFillColor(colors.HexColor("#0F172A"))
    c.setFont("Helvetica-Bold", 11)
    c.drawString(info_x, y + box_h - 1.3 * cm, tag_label)
    
    # Description
    c.setFillColor(colors.HexColor("#475569"))
    c.setFont("Helvetica", 8)
    text_obj = c.beginText(info_x, y + box_h - 1.9 * cm)
    text_obj.setLeading(11)
    for line in description:
        text_obj.textLine(line)
    c.drawText(text_obj)
    
    # Mounting tip
    c.setFillColor(colors.HexColor("#047857"))
    c.setFont("Helvetica-Bold", 7.5)
    c.drawString(info_x, y + 0.5 * cm, "✂ CUT ALONG DASHED BOUNDARY")
    c.setFillColor(colors.HexColor("#64748B"))
    c.setFont("Helvetica", 7)
    c.drawString(info_x, y + 0.1 * cm, "Mount vertically at car camera height (~12-15 cm from ground)")
    
    # Calibration ruler underneath marker
    draw_calibration_ruler(c, x, y - 1.1 * cm, length_cm=marker_size_cm)
    
    c.restoreState()

def create_aruco_pdf(output_pdf_path):
    os.makedirs(os.path.dirname(os.path.abspath(output_pdf_path)), exist_ok=True)
    c = canvas.Canvas(output_pdf_path, pagesize=A4)
    width, height = A4
    
    temp_dir = tempfile.mkdtemp()
    dict_type = cv2.aruco.DICT_4X4_50
    
    marker_files = {}
    for tag_id in range(6):
        img = generate_marker_image(dict_type, tag_id, pixel_size=1000)
        p = os.path.join(temp_dir, f"aruco_{tag_id}.png")
        cv2.imwrite(p, img)
        marker_files[tag_id] = p
        
    tag_metadata = {
        0: {
            "label": "HOME BASE / ORIGIN DOCK",
            "desc": [
                "Primary vehicle start point & charging dock.",
                "Used for return-to-base and origin alignment.",
                "Vehicle stops when distance <= 28 cm.",
                "Dictionary: DICT_4X4_50 | Marker: 10.0 cm"
            ]
        },
        1: {
            "label": "WAYPOINT ALPHA (MIDPOINT)",
            "desc": [
                "Corridor midpoint & navigation check-in.",
                "Provides bearing correction on long straights.",
                "Autonomously sequenced in linked route mode.",
                "Dictionary: DICT_4X4_50 | Marker: 10.0 cm"
            ]
        },
        2: {
            "label": "WAYPOINT BETA (DETOUR PIVOT)",
            "desc": [
                "Turn pivot & obstacle detour re-alignment tag.",
                "Placed before junctions, doorways, or turns.",
                "Vehicle navigates smoothly into target heading.",
                "Dictionary: DICT_4X4_50 | Marker: 10.0 cm"
            ]
        },
        3: {
            "label": "TERMINAL (DESTINATION GOAL)",
            "desc": [
                "Final mission destination goal landmark.",
                "Vehicle homes in with millimeter 3D precision.",
                "Triggers ARRIVED_AT_DESTINATION state & halt.",
                "Dictionary: DICT_4X4_50 | Marker: 10.0 cm"
            ]
        },
        4: {
            "label": "OBSTACLE BYPASS TARGET",
            "desc": [
                "Alternate bypass beacon around physical blocks.",
                "Used by ONNX and radar detour planners.",
                "Dictionary: DICT_4X4_50 | Marker: 10.0 cm"
            ]
        },
        5: {
            "label": "AUXILIARY / SECONDARY DOCK",
            "desc": [
                "Secondary docking station or waypoint link.",
                "Dictionary: DICT_4X4_50 | Marker: 10.0 cm"
            ]
        }
    }
    
    # ── Page 1: Tags 0 & 1 ────────────────────────────────────────────────────
    draw_header(c, "Autonomous RC Car — Waypoint Navigation Markers (10x10 cm)", 1, total_pages=4)
    draw_marker_card(c, marker_files[0], 1.5 * cm, height - 13.8 * cm, 0,
                     tag_metadata[0]["label"], tag_metadata[0]["desc"], marker_size_cm=10.0)
    draw_marker_card(c, marker_files[1], 1.5 * cm, height - 26.6 * cm, 1,
                     tag_metadata[1]["label"], tag_metadata[1]["desc"], marker_size_cm=10.0)
    c.showPage()
    
    # ── Page 2: Tags 2 & 3 ────────────────────────────────────────────────────
    draw_header(c, "Autonomous RC Car — Waypoint Navigation Markers (10x10 cm)", 2, total_pages=4)
    draw_marker_card(c, marker_files[2], 1.5 * cm, height - 13.8 * cm, 2,
                     tag_metadata[2]["label"], tag_metadata[2]["desc"], marker_size_cm=10.0)
    draw_marker_card(c, marker_files[3], 1.5 * cm, height - 26.6 * cm, 3,
                     tag_metadata[3]["label"], tag_metadata[3]["desc"], marker_size_cm=10.0)
    c.showPage()
    
    # ── Page 3: Tags 4 & 5 ────────────────────────────────────────────────────
    draw_header(c, "Autonomous RC Car — Auxiliary Waypoint Markers (10x10 cm)", 3, total_pages=4)
    draw_marker_card(c, marker_files[4], 1.5 * cm, height - 13.8 * cm, 4,
                     tag_metadata[4]["label"], tag_metadata[4]["desc"], marker_size_cm=10.0)
    draw_marker_card(c, marker_files[5], 1.5 * cm, height - 26.6 * cm, 5,
                     tag_metadata[5]["label"], tag_metadata[5]["desc"], marker_size_cm=10.0)
    c.showPage()
    
    # ── Page 4: 4-in-1 Compact Sheet (7.0 cm tags) ────────────────────────────
    draw_header(c, "Autonomous RC Car — 4-in-1 Compact Testing Sheet (7.0 cm)", 4, total_pages=4)
    
    c.saveState()
    c.setFont("Helvetica-Bold", 9)
    c.setFillColor(colors.HexColor("#1E293B"))
    c.drawString(1.5 * cm, height - 2.8 * cm, "Quick 4-in-1 Sheet: Scaled to 7.0 cm for desktop test tracks or small cardboard setups.")
    
    compact_size = 7.0 * cm
    grid_coords = [
        (1.5 * cm, height - 13.2 * cm, 0, "TAG #0 (Home Base)"),
        (11.0 * cm, height - 13.2 * cm, 1, "TAG #1 (Waypoint Alpha)"),
        (1.5 * cm, height - 25.8 * cm, 2, "TAG #2 (Waypoint Beta)"),
        (11.0 * cm, height - 25.8 * cm, 3, "TAG #3 (Destination)")
    ]
    
    for gx, gy, tid, ttitle in grid_coords:
        c.drawImage(marker_files[tid], gx, gy, width=compact_size, height=compact_size, preserveAspectRatio=True)
        # Dashed border
        c.setStrokeColor(colors.HexColor("#94A3B8"))
        c.setLineWidth(0.8)
        c.setDash(3, 3)
        c.rect(gx, gy, compact_size, compact_size, stroke=1, fill=0)
        c.setDash([])
        # Badge label
        c.setFillColor(colors.HexColor("#4F46E5"))
        c.roundRect(gx, gy + compact_size + 0.15 * cm, 4.6 * cm, 0.55 * cm, 3, stroke=0, fill=1)
        c.setFillColor(colors.white)
        c.setFont("Helvetica-Bold", 8)
        c.drawString(gx + 0.25 * cm, gy + compact_size + 0.30 * cm, ttitle)
        draw_calibration_ruler(c, gx, gy - 0.9 * cm, length_cm=7.0)
        
    c.restoreState()
    c.showPage()
    
    c.save()
    print(f"[SUCCESS] ArUco markers PDF created at: {output_pdf_path}")

if __name__ == "__main__":
    out_pdf = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "aruco_markers.pdf")
    create_aruco_pdf(out_pdf)
