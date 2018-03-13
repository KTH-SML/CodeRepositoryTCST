#ifndef FORMULA_PARSER_HPP
#define FORMULA_PARSER_HPP

#include "exprtk.hpp"
#include <string>

template <class T>
class FormulaParser{
public:
	static exprtk::expression<T> expression(std::string formula, std::vector<T>& param, std::string param_name){

		exprtk::symbol_table<T> symbol_table;
		symbol_table.add_vector(param_name, param);

		exprtk::expression<T> expression;
		expression.register_symbol_table(symbol_table);

		exprtk::parser<T> parser;
		parser.compile(formula, expression);

		return expression;
	}
};

#endif
