<?xml version="1.0" encoding="UTF-8"?>
<xsl:stylesheet version="1.0"
    xmlns:xsl="http://www.w3.org/1999/XSL/Transform">

    <!--
      Link summary XSLT
      ------------------
      Usage:
        xsltproc tools/urdf_link_report.xslt path/to/robot.urdf > link_report.html
      The transform scans all <link> elements in a URDF/Xacro-expanded file
      and produces a compact HTML table with the key properties for each link.
    -->

    <xsl:output method="html" indent="yes" encoding="UTF-8"
        doctype-public="-//W3C//DTD HTML 4.01 Transitional//EN"
        doctype-system="http://www.w3.org/TR/html4/loose.dtd"/>

    <!-- Default template: render report wrapper -->
    <xsl:template match="/robot">
        <html>
            <head>
                <title>
                    <xsl:text>URDF Link Report - </xsl:text>
                    <xsl:value-of select="@name"/>
                </title>
                <style type="text/css">
                    body { font-family: Arial, sans-serif; margin: 1.5em; }
                    h1 { font-size: 1.6em; }
                    table { border-collapse: collapse; width: 100%; }
                    th, td { border: 1px solid #ccc; padding: 0.4em 0.6em; }
                    th { background: #f2f2f2; text-align: left; }
                    tr:nth-child(even) { background: #fafafa; }
                    code { font-size: 0.95em; }
                </style>
            </head>
            <body>
                <h1>
                    <xsl:text>Link Summary for </xsl:text>
                    <xsl:value-of select="@name"/>
                </h1>
                <p>Total links: <strong><xsl:value-of select="count(link)"/></strong></p>
                <table>
                    <thead>
                        <tr>
                            <th>#</th>
                            <th>Link Name</th>
                            <th>Mass (kg)</th>
                            <th>Inertial Origin xyz (m)</th>
                            <th>Visual Geometry</th>
                        </tr>
                    </thead>
                    <tbody>
                        <xsl:apply-templates select="link">
                            <xsl:sort select="@name"/>
                        </xsl:apply-templates>
                    </tbody>
                </table>
            </body>
        </html>
    </xsl:template>

    <!-- Per-link row -->
    <xsl:template match="link">
        <tr>
            <td><xsl:value-of select="position()"/></td>
            <td><code><xsl:value-of select="@name"/></code></td>
            <td>
                <xsl:choose>
                    <xsl:when test="inertial/mass/@value">
                        <xsl:value-of select="format-number(inertial/mass/@value, '0.###')"/>
                    </xsl:when>
                    <xsl:otherwise>-</xsl:otherwise>
                </xsl:choose>
            </td>
            <td>
                <xsl:choose>
                    <xsl:when test="inertial/origin/@xyz">
                        <code><xsl:value-of select="inertial/origin/@xyz"/></code>
                    </xsl:when>
                    <xsl:otherwise>-</xsl:otherwise>
                </xsl:choose>
            </td>
            <td>
                <xsl:choose>
                    <xsl:when test="visual/geometry/mesh/@filename">
                        <code><xsl:value-of select="visual/geometry/mesh/@filename"/></code>
                    </xsl:when>
                    <xsl:when test="visual/geometry/*">
                        <xsl:value-of select="name(visual/geometry/*[1])"/>
                    </xsl:when>
                    <xsl:otherwise>-</xsl:otherwise>
                </xsl:choose>
            </td>
        </tr>
    </xsl:template>

</xsl:stylesheet>
