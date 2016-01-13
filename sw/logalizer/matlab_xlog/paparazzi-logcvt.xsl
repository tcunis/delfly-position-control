<?xml version="1.0" encoding="windows-1252"?>
<xsl:stylesheet version="2.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform" xmlns:fo="http://www.w3.org/1999/XSL/Format" xmlns:xs="http://www.w3.org/2001/XMLSchema" xmlns:fn="http://www.w3.org/2005/xpath-functions">
	<xsl:output method="xml" encoding="windows-1252" indent="yes" exclude-result-prefixes="xsl fo xs fn"/>
	<xsl:param name="data-encoding" select='"windows-1252"'/>
	<xsl:key name="msg-definition" match="/configuration/protocol/msg_class/message" use="@NAME"/>
	<xsl:template match="configuration[@data_file]">
		<xsl:variable name="root" select="."/>
		<xsl:variable name="data-lines" select="unparsed-text(@data_file,$data-encoding)"/>
		<message-log filename="{substring-before(@data_file,'.data')}">
			<xsl:copy-of select="@time_of_day"/>
			<xsl:copy-of select="protocol"/>
			<logdata>
				<xsl:analyze-string select="$data-lines" regex="\n">
					<xsl:non-matching-substring>
						<xsl:variable name="data-tokens" select="tokenize(normalize-space(.),' ')"/>
						<xsl:variable name="msg-name" select="$data-tokens[3]"/>
						<xsl:apply-templates select="key('msg-definition',$msg-name,$root)">
							<xsl:with-param name="msg-name" select="$msg-name"/>
							<xsl:with-param name="time" select="$data-tokens[1]"/>
							<xsl:with-param name="ac-id" select="$data-tokens[2]"/>
							<xsl:with-param name="data-values" select="subsequence($data-tokens,4)"/>
						</xsl:apply-templates>
					</xsl:non-matching-substring>
				</xsl:analyze-string>
			</logdata>
		</message-log>
	</xsl:template>
	<xsl:template match="message">
		<xsl:param name="msg-name"/>
		<xsl:param name="time"/>
		<xsl:param name="ac-id"/>
		<xsl:param name="data-values"/>
		<log-entry name="{$msg-name}" time="{$time}" ac-id="{$ac-id}">
			<xsl:for-each select="field">
				<xsl:variable name="raw-value" select="subsequence($data-values,position(),1)"/>
				<field name="{@NAME}">
					<xsl:apply-templates select=".">
						<xsl:with-param name="raw-value" select="$raw-value"/>
					</xsl:apply-templates>
				</field>
			</xsl:for-each>
		</log-entry>
	</xsl:template>
	<xsl:template match="field[@VALUES]">
		<xsl:param name="raw-value"/>
		<xsl:attribute name="raw-value" select="$raw-value"/>
		<value>
			<xsl:value-of select="subsequence(tokenize(@VALUES,'\|'),xs:integer($raw-value) + 1,1)"/>
		</value>
	</xsl:template>
	<xsl:template match="field[@ALT_UNIT_COEF]">
		<xsl:param name="raw-value"/>
		<xsl:choose>
			<xsl:when test="@UNIT">
				<value unit="{normalize-space(@UNIT)}">
					<xsl:value-of select="$raw-value"/>
				</value>
			</xsl:when>
			<xsl:otherwise>
				<xsl:attribute name="raw-value" select="$raw-value"/>
			</xsl:otherwise>
		</xsl:choose>
		<value>
			<xsl:if test="@ALT_UNIT">
				<xsl:attribute name="unit" select="normalize-space(@ALT_UNIT)"/>
			</xsl:if>
			<xsl:value-of select="xs:double($raw-value) * xs:double(@ALT_UNIT_COEF)"/>
		</value>
	</xsl:template>
	<xsl:template match="field">
		<xsl:param name="raw-value"/>
		<value>
			<xsl:if test="@UNIT">
				<xsl:attribute name="unit" select="normalize-space(@UNIT)"/>
			</xsl:if>
			<xsl:value-of select="$raw-value"/>
		</value>
	</xsl:template>
</xsl:stylesheet>
